#!/usr/bin/env python3
import argparse
import datetime
import json
import os
import re
import selectors
import shutil
import signal
import statistics
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

EXPECTED_MODELS = {
    "ground_plane",
    "smarty",
    "road",
    "speed_limit_zone_30_start_sign_0",
    "speed_limit_zone_30_end_sign_0",
    "turn_right_sign_0",
    "turn_right_sign_1",
    "turn_left_sign_0",
    "turn_left_sign_1",
    "zebra_crossing_sign_0",
    "parking_sign_0",
    "obstacle_0",
    "obstacle_1",
}
EXPECTED_NODES = {
    "car_control_bridge",
    "car_sensor_bridge",
    "robot_description_publisher",
    "ros_gz_bridge",
    "world_spawner",
}


def executable(name: str) -> str:
    path = shutil.which(name)
    if path is None:
        raise FileNotFoundError(f"Required executable not found: {name}")
    return path


def power_state() -> dict[str, str | None]:
    paths = {
        "ac_online": "/sys/class/power_supply/ACAD/online",
        "battery_percent": "/sys/class/power_supply/BAT1/capacity",
        "battery_status": "/sys/class/power_supply/BAT1/status",
    }
    return {
        name: Path(path).read_text().strip() if Path(path).exists() else None
        for name, path in paths.items()
    }


class Observer(Node):
    def __init__(self) -> None:
        super().__init__("baseline_observer", enable_rosout=False)
        self.create_subscription(
            Clock, "/clock", self.on_clock, qos_profile_sensor_data
        )
        self.last_clock = None
        self.advancing = False
        self.ready_at = None
        self.stop_requested = False

    def on_clock(self, msg: Clock) -> None:
        current = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if self.last_clock is not None and current > self.last_clock:
            self.advancing = True
        self.last_clock = current

    def run(self) -> None:
        while not self.stop_requested and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.ready_at is None and self.advancing:
                nodes = set(self.get_node_names())
                if nodes >= EXPECTED_NODES:
                    self.ready_at = time.monotonic()


def stats_sample(duration: float) -> list[dict[str, float]]:
    process = subprocess.Popen(  # noqa: S603
        [executable("gz"), "topic", "-e", "-t", "/world/smartrollerz/stats"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if process.stdout is None or process.stderr is None:
        process.terminate()
        process.wait()
        raise RuntimeError("Gazebo statistics pipes were not created")
    stdout = process.stdout
    stderr = process.stderr
    selector = selectors.DefaultSelector()
    selector.register(stdout, selectors.EVENT_READ)
    started = time.monotonic()
    buffer = ""
    samples = []
    try:
        while time.monotonic() - started < duration:
            if process.poll() is not None:
                raise RuntimeError(
                    f"Gazebo stats probe exited: {stderr.read().decode()}"
                )
            events = selector.select(timeout=0.25)
            if not events:
                continue
            buffer += os.read(stdout.fileno(), 65536).decode()
            while "\n\n" in buffer:
                message, buffer = buffer.split("\n\n", 1)
                sim = re.search(
                    r"sim_time\s*\{\s*(?:sec:\s*(\d+))?\s*(?:nsec:\s*(\d+))?",
                    message,
                )
                rtf = re.search(r"real_time_factor:\s*([0-9.eE+-]+)", message)
                if sim and rtf:
                    value = int(sim.group(1) or 0) + int(sim.group(2) or 0) / 1e9
                    samples.append(
                        {
                            "wall_s": time.monotonic() - started,
                            "sim_s": value,
                            "rtf": float(rtf.group(1)),
                        }
                    )
    finally:
        selector.close()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    minimum_samples = 2
    if len(samples) < minimum_samples:
        raise RuntimeError(f"Insufficient Gazebo stats samples: {len(samples)}")
    elapsed = samples[-1]["wall_s"] - samples[0]["wall_s"]
    if elapsed < duration * 0.9:
        raise RuntimeError(f"Only {elapsed:.1f}s of valid stats observed")
    return samples


def process_details(pid: int) -> tuple[int, str, int] | None:
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return int(fields[1]), fields[0], int(fields[19])
    except (FileNotFoundError, ProcessLookupError, IndexError):
        return None


def descendants(pid: int) -> dict[int, int]:
    processes = {}
    for path in Path("/proc").iterdir():
        if path.name.isdigit():
            details = process_details(int(path.name))
            if details is not None:
                processes[int(path.name)] = details
    children = {pid}
    previous = set()
    while children != previous:
        previous = children.copy()
        children.update(p for p, details in processes.items() if details[0] in children)
    return {p: processes[p][2] for p in children if p in processes}


def stop_launch(process: subprocess.Popen[bytes]) -> None:
    started_processes = descendants(process.pid)
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=25)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
    for pid, start_time in started_processes.items():
        details = process_details(pid)
        if details and details[1] != "Z" and details[2] == start_time:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                continue
    deadline = time.monotonic() + 10
    remaining = []
    while time.monotonic() < deadline:
        remaining = [
            pid
            for pid, start_time in started_processes.items()
            if (details := process_details(pid))
            and details[1] != "Z"
            and details[2] == start_time
        ]
        if not remaining:
            return
        time.sleep(0.2)
    raise RuntimeError(f"Benchmark child processes did not exit: {remaining}")


def collect_run(
    observer: Observer,
    process: subprocess.Popen[bytes],
    start: float,
    info: dict[str, object],
) -> None:
    deadline = start + 90
    scene_at = None
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"Launch exited prematurely with {process.returncode}")
        try:
            result = subprocess.run(  # noqa: S603
                [executable("gz"), "model", "--list"],
                capture_output=True,
                text=True,
                timeout=5,
                check=False,
            )
            models = set(re.findall(r"^\s*-\s+(\S+)\s*$", result.stdout, re.MULTILINE))
            if result.returncode == 0 and models >= EXPECTED_MODELS:
                scene_at = time.monotonic()
                info["models_seen"] = sorted(models)
                break
        except subprocess.TimeoutExpired:
            pass
        time.sleep(1)
    if scene_at is None:
        raise RuntimeError("Scene was not verified within 90 seconds")
    while observer.ready_at is None and time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"Launch exited prematurely with {process.returncode}")
        time.sleep(0.1)
    if observer.ready_at is None:
        raise RuntimeError("ROS node and advancing-clock readiness timed out")
    scene_ready = max(scene_at, observer.ready_at)
    info["ros_ready_s"] = observer.ready_at - start
    info["scene_ready_s"] = scene_ready - start
    while time.monotonic() < scene_ready + 30:
        if process.poll() is not None:
            raise RuntimeError(f"Launch exited during warmup with {process.returncode}")
        time.sleep(0.25)
    info["load_before_stats"] = os.getloadavg()
    samples = stats_sample(60)
    info["samples"] = samples
    if process.poll() is not None:
        raise RuntimeError(f"Launch exited during sampling with {process.returncode}")
    info["window_rtf"] = (samples[-1]["sim_s"] - samples[0]["sim_s"]) / (
        samples[-1]["wall_s"] - samples[0]["wall_s"]
    )
    info["sample_median_rtf"] = statistics.median(x["rtf"] for x in samples)
    info["sample_min_rtf"] = min(x["rtf"] for x in samples)
    info["load_after_stats"] = os.getloadavg()
    info["power_after"] = power_state()


def run_one(number: int, output: Path) -> bool:
    output.mkdir(parents=True, exist_ok=True)
    os.environ["GZ_PARTITION"] = f"smarty_baseline_{os.getpid()}_{number}"
    observer = Observer()
    observer_thread = threading.Thread(target=observer.run)
    observer_thread.start()
    info: dict[str, object] = {
        "run": number,
        "start_local": datetime.datetime.now().astimezone().isoformat(),
        "load_before": os.getloadavg(),
        "power_before": power_state(),
        "expected_models": sorted(EXPECTED_MODELS),
        "status": "failed",
    }
    process = None
    log_path = output / f"launch-{number}.log"
    try:
        with log_path.open("w") as log:
            start = time.monotonic()
            process = subprocess.Popen(  # noqa: S603
                [
                    executable("ros2"),
                    "launch",
                    "simulation",
                    "simulation.launch.py",
                    "rviz:=false",
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            collect_run(observer, process, start, info)
            info["status"] = "success"
    except (OSError, RuntimeError, subprocess.SubprocessError, ValueError) as exc:
        info["status"] = "failed"
        info["error"] = str(exc)
    finally:
        if process is not None:
            try:
                stop_launch(process)
            except (OSError, RuntimeError) as exc:
                info["status"] = "failed"
                info["error"] = str(exc)
        observer.stop_requested = True
        observer_thread.join(timeout=3)
        observer.destroy_node()
        info["end_local"] = datetime.datetime.now().astimezone().isoformat()
        (output / f"run-{number}.json").write_text(json.dumps(info, indent=2) + "\n")
    print(
        json.dumps({key: value for key, value in info.items() if key != "samples"}),
        flush=True,
    )
    return info["status"] == "success"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    try:
        for number in range(1, args.runs + 1):
            if not run_one(number, args.output):
                raise SystemExit(1)
            if number < args.runs:
                time.sleep(30)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
