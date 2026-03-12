import json
import os
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
import shapely
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from simulation.groundtruth.road import road

# =====================================================================
# Math and Geometry Utilities
# =====================================================================


def get_rigid_inverse_matrix(trans: TransformStamped) -> np.ndarray:
    """Faster way to invert a rigid transform without np.linalg.inv."""
    x = trans.transform.rotation.x
    y = trans.transform.rotation.y
    z = trans.transform.rotation.z
    w = trans.transform.rotation.w
    tx = trans.transform.translation.x
    ty = trans.transform.translation.y
    tz = trans.transform.translation.z

    # 1. Quaternion to Rotation Matrix R
    R = np.array(
        [
            [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y],
            [2 * x * y + 2 * w * z, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * w * x],
            [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x * x - 2 * y * y],
        ]
    )

    # 2. Inverse of Rigid Matrix [R | t] is [R^T | -R^T * t]
    RT = R.T
    neg_RT_t = -RT @ np.array([tx, ty, tz])

    inv_mat = np.eye(4)
    inv_mat[:3, :3] = RT
    inv_mat[:3, 3] = neg_RT_t
    return inv_mat


def filter_on_image(pts_arr: np.ndarray, w: int, h: int) -> np.ndarray:
    """Filters points array to keep those within image dimensions."""
    if pts_arr.shape[0] == 0:
        return np.zeros((0, 2))
    valid_mask = (
        (pts_arr[:, 0] >= 0)
        & (pts_arr[:, 0] < w)
        & (pts_arr[:, 1] >= 0)
        & (pts_arr[:, 1] < h)
    )
    return pts_arr[valid_mask]


def cull_points(
    points: np.ndarray, car_x: float, car_y: float, cull_radius: float
) -> np.ndarray:
    """Culls map points to radius around the car."""
    dist_sq = (points[0] - car_x) ** 2 + (points[1] - car_y) ** 2
    return points[:, dist_sq <= cull_radius**2]


def split_contiguous_segments(
    pts: np.ndarray, max_jump_px: float = 60.0
) -> list[np.ndarray]:
    """Split a pixel-space polyline at large gaps and return all contiguous segments."""
    if pts.shape[0] < 2:
        return [pts]

    diffs = np.diff(pts, axis=0)
    jump_dist = np.hypot(diffs[:, 0], diffs[:, 1])
    split_indices = np.where(jump_dist > max_jump_px)[0] + 1
    return np.split(pts, split_indices)


def draw_poly(img: np.ndarray, pts: np.ndarray, color: Tuple[int, int, int]):
    """Draws all contiguous polyline segments onto a cv2 image.
    Automatically avoids stitching disconnected road sections with long jumps."""
    if pts is None or pts.size == 0:
        return

    for segment in split_contiguous_segments(pts):
        if segment.shape[0] < 2:
            continue
        poly = segment.astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(img, [poly], isClosed=False, color=color, thickness=3)


# =====================================================================
# Main Node Class
# =====================================================================


class GroundtruthLabeler(Node):
    """Ground truth labeler node for extracting lane polylines."""

    def __init__(self):
        super().__init__("groundtruth_labeler")

        self.package_share_path = get_package_share_directory("simulation")

        self._setup_parameters()
        self._setup_directories()

        # Load road and map points
        self.road_map = road.load(self.road_path)
        (
            self.ml_arr,
            self.ll_arr,
            self.rl_arr,
            self.ml_s,
            self.ll_s,
            self.rl_s,
        ) = self._precompute_road_points()

        # State and Caches
        self._last_end_time = None
        self._last_car_s = None
        self._static_tf_cache = {}
        self.camera_info = None
        self.camera_k = None
        self.camera_d = None
        self._log_once = True

        # ROS Publishers / Subscribers / Subsystems setup
        self.cv_bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._setup_ros_communications()

        self.get_logger().info("GroundTruth labeler initialized.")

    def _setup_parameters(self):
        """Declares and fetches node parameters."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("road", "models/roads/default_road"),
                ("output_dir", "output"),
                ("lookahead_distance", 5.0),
                ("lookbehind_distance", 5.0),
                ("right_lookahead_margin", 1.0),
                ("save_data", False),
            ],
        )

        self.road_path = os.path.join(
            self.package_share_path,
            self.get_parameter("road").value,
            "road.py",
        )
        self.output_dir = self.get_parameter("output_dir").value
        self.lookahead_distance = self.get_parameter("lookahead_distance").value
        self.lookbehind_distance = self.get_parameter("lookbehind_distance").value
        self.right_lookahead_margin = self.get_parameter(
            "right_lookahead_margin"
        ).value
        self.save_data = self.get_parameter("save_data").value

    def _setup_directories(self):
        """Creates output directories for data saving."""
        self.img_dir = os.path.join(self.output_dir, "images")
        self.debug_dir = os.path.join(self.output_dir, "debug_images")
        self.labels_dir = os.path.join(self.output_dir, "labels")

        os.makedirs(self.img_dir, exist_ok=True)
        os.makedirs(self.debug_dir, exist_ok=True)
        os.makedirs(self.labels_dir, exist_ok=True)

        self.get_logger().info(f"Saving output to {self.output_dir}")

    def _setup_ros_communications(self):
        """Initializes ROS subscribers and publishers."""
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera/info",
            self.camera_info_callback,
            1,
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = self.create_subscription(
            Image,
            "/camera/image/raw",
            self.image_callback,
            qos,
        )

        self.debug_image_pub = self.create_publisher(
            Image, "/groundtruth/debug_image", 1
        )

    def _precompute_road_points(
        self,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Samples the road once at 5cm intervals and precomputes each point's
        arc-length along the road centerline for true along-track filtering."""
        self.get_logger().info("Precomputing dense road points into Numpy matrices...")
        step = 0.05
        all_ml, all_ll, all_rl = [], [], []
        all_ml_s, all_ll_s, all_rl_s = [], [], []

        for section in self.road_map.sections:
            ml = shapely.segmentize(section.middle_line._linestring, step)
            ll = shapely.segmentize(section.left_line._linestring, step)
            rl = shapely.segmentize(section.right_line._linestring, step)
            sec_ml = section.middle_line._linestring
            sec_s0 = section.prev_length

            for pt in ml.coords:
                all_ml.append([pt[0], pt[1], 0.0, 1.0])
                all_ml_s.append(sec_s0 + sec_ml.project(shapely.Point(pt[0], pt[1])))
            for pt in ll.coords:
                all_ll.append([pt[0], pt[1], 0.0, 1.0])
                all_ll_s.append(sec_s0 + sec_ml.project(shapely.Point(pt[0], pt[1])))
            for pt in rl.coords:
                all_rl.append([pt[0], pt[1], 0.0, 1.0])
                all_rl_s.append(sec_s0 + sec_ml.project(shapely.Point(pt[0], pt[1])))

        return (
            np.array(all_ml).T,
            np.array(all_ll).T,
            np.array(all_rl).T,
            np.array(all_ml_s),
            np.array(all_ll_s),
            np.array(all_rl_s),
        )

    def _get_car_arc_length(self, car_x: float, car_y: float) -> Optional[float]:
        """Project the car position onto the nearest centerline section and return
        global road arc-length coordinate (meters from road start)."""
        car_point = shapely.Point(car_x, car_y)

        candidates = []
        for section in self.road_map.sections:
            sec_ml = section.middle_line._linestring
            sec_proj = sec_ml.project(car_point)
            sec_dist = sec_ml.distance(car_point)
            candidates.append((sec_dist, section.prev_length + sec_proj))

        if not candidates:
            return None

        # Geometric closest section fallback
        closest_dist, closest_s = min(candidates, key=lambda c: c[0])

        # Temporal continuity: prevent jumps to a different nearby section at
        # intersections which can clip the behind-car window unexpectedly.
        if self._last_car_s is None:
            self._last_car_s = closest_s
            return closest_s

        max_s_jump = 3.0  # [m] plausible movement between consecutive frames
        continuity_candidates = [
            (dist, s)
            for dist, s in candidates
            if abs(s - self._last_car_s) <= max_s_jump
        ]

        if continuity_candidates:
            _, chosen_s = min(continuity_candidates, key=lambda c: c[0])
        else:
            # Large discontinuity (e.g. spawn/reset) -> accept geometric closest.
            # Also avoid locking onto stale history when closest match is very clear.
            chosen_s = closest_s if closest_dist < 0.5 else self._last_car_s

        self._last_car_s = chosen_s
        return chosen_s

    def camera_info_callback(self, msg: CameraInfo):
        """Update active camera intrinsic metrics on info message."""
        if self.camera_info is None:
            self.camera_info = msg
            self.camera_k = np.array(msg.k).reshape((3, 3))
            self.camera_d = (
                np.array(msg.d[:4]).reshape((1, 4))
                if len(msg.d) >= 4
                else np.zeros((1, 4))
            )
            self.get_logger().info(
                f"Received CameraInfo. Frame ID: {msg.header.frame_id}, D={self.camera_d}"
            )

    def _get_static_transform(
        self, target_frame: str, source_frame: str
    ) -> Optional[TransformStamped]:
        """Lookup a static transform, cached after the first successful lookup."""
        tf_key = (target_frame, source_frame)
        if tf_key in self._static_tf_cache:
            return self._static_tf_cache[tf_key]
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            self._static_tf_cache[tf_key] = trans
            return trans
        except Exception as e:
            self.get_logger().warning(f"Static TF lookup failed ({source_frame}->{target_frame}): {e}")
            return None

    def _get_dynamic_transform(
        self, target_frame: str, source_frame: str, stamp: Time
    ) -> Optional[TransformStamped]:
        """Lookup a dynamic transform at the exact image timestamp.
        Waits up to 200ms for TF data to arrive. If the only issue is extrapolation
        and the gap to the latest available pose is within the acceptable threshold
        (50ms), falls back to that pose silently — errors that small are negligible.
        Larger gaps are dropped to avoid shift artifacts."""
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except Exception as e:
            if "extrapolation into the future" not in str(e):
                self.get_logger().warning(
                    f"Dynamic TF lookup failed ({source_frame}->{target_frame}): {e}"
                )
                return None

            # Extrapolation: check how far ahead the image stamp is from the latest TF
            try:
                latest = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05),
                )
                latest_stamp = Time.from_msg(latest.header.stamp)
                gap_ms = (stamp.nanoseconds - latest_stamp.nanoseconds) / 1e6

                if gap_ms <= 50.0:
                    # Gap is within one TF cycle — positional error is negligible
                    return latest

                self.get_logger().warning(
                    f"TF gap too large to use fallback ({gap_ms:.0f}ms) "
                    f"for {source_frame}->{target_frame}, dropping frame"
                )
                return None
            except Exception as e2:
                self.get_logger().warning(
                    f"TF fallback also failed ({source_frame}->{target_frame}): {e2}"
                )
                return None

    def image_callback(self, msg: Image):
        """Process an image directly. By using a single-threaded executor and QoS depth=1, DDS auto-drops images produced while blocked."""
        start_time = time.perf_counter()

        if self._last_end_time is not None:
            time_since_last = start_time - self._last_end_time
            self.get_logger().info(f"Waited {time_since_last * 1000:.1f}ms for new image since last computation ended")

        try:
            self.get_logger().info(
                f"Starting computation for image {msg.header.stamp.sec}_{msg.header.stamp.nanosec} at {start_time:.3f}"
            )
            self._process_image(msg)
        finally:
            end_time = time.perf_counter()
            self._last_end_time = end_time
            self.get_logger().info(
                f"Finished computation for image {msg.header.stamp.sec}_{msg.header.stamp.nanosec} at {end_time:.3f}. Duration: {(end_time - start_time) * 1000:.1f}ms"
            )

    def _compute_pixels(self, cam_pts: np.ndarray, w: int, h: int) -> np.ndarray:
        """Projects 3D points from the camera frame to 2D image pixels based on active camera config."""
        if cam_pts.shape[1] == 0:
            return np.zeros((0, 2))

        # Remap to optical frame: X=right, Y=down, Z=forward
        pts_opt = np.vstack([-cam_pts[1, :], -cam_pts[2, :], cam_pts[0, :]])

        X, Y, Z = pts_opt[0, :], pts_opt[1, :], pts_opt[2, :]

        # Cull points behind the camera
        valid = Z > 0.01
        if not np.any(valid):
            return np.zeros((0, 2))

        X, Y, Z = X[valid], Y[valid], Z[valid]

        # Camera Intrinsics handling
        cx, cy = w / 2.0, h / 2.0
        hfov = 1.858

        f_calc = (w / 2.0) / np.sin(hfov / 2.0)

        fx = fy = f_calc

        # Output orthographic projection
        depth_norm = np.sqrt(X**2 + Y**2 + Z**2)
        u = cx + fx * (X / depth_norm)
        v = cy + fy * (Y / depth_norm)
        pts_2d = np.column_stack([u, v])

        # Optional debug logging for initialization tracking
        if self._log_once:
            self.get_logger().info("--- PROJECTION DEBUG V3 ---")
            self.get_logger().info(f"Image Size: {w}x{h}")
            self.get_logger().info("Model: orthographic")
            self.get_logger().info(
                f"Calculated K: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}"
            )
            if len(u) > 0:
                self.get_logger().info(
                    f"First Point: Optical(X={X[0]:.3f}, Y={Y[0]:.3f}, Z={Z[0]:.3f}) -> Pixel(u={u[0]:.1f}, v={v[0]:.1f})"
                )
            self.get_logger().info("---------------------------")
            self._log_once = False

        return pts_2d

    def _process_image(self, msg: Image):
        """Performs full spatial alignment mapping 2D frames over valid 3D tracks."""
        t0 = time.perf_counter()

        # Convert the image stamp to rclpy.time.Time for correct TF lookup
        img_time = Time.from_msg(msg.header.stamp)

        # Gazebo appends '/camera' for the optical frame (e.g. 'smarty/camera_link/camera')
        # but the TF tree only has the physical link frame. Strip the suffix as fallback.
        cam_frame = msg.header.frame_id
        if not self.tf_buffer.can_transform("smarty", cam_frame, rclpy.time.Time()):
            cam_frame = cam_frame.rsplit("/", 1)[0]

        # Dynamic: must use exact image timestamp so the pose matches the captured frame
        world_to_car = self._get_dynamic_transform("world", "smarty", img_time)
        # Static: camera is rigidly mounted, cache it after the first lookup
        car_to_cam = self._get_static_transform("smarty", cam_frame)

        if world_to_car is None or car_to_cam is None:
            return

        t_tf = time.perf_counter()

        # Matrix derivations
        world_to_car_mat = get_rigid_inverse_matrix(world_to_car)
        car_to_cam_mat = get_rigid_inverse_matrix(car_to_cam)
        cam_matrix = car_to_cam_mat @ world_to_car_mat

        # Parse OpenCV structures
        if msg.encoding in ["mono8", "8UC1"]:
            cv_img_gray = self.cv_bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
            cv_img = cv2.cvtColor(cv_img_gray, cv2.COLOR_GRAY2BGR)
        else:
            cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        h, w = cv_img.shape[:2]

        # Cull radius: generous enough to include both configured look-ahead and
        # look-behind ranges before along-track masking is applied.
        car_pos_world = world_to_car.transform.translation
        max_track_window = max(
            self.lookbehind_distance,
            self.lookahead_distance + self.right_lookahead_margin,
        )
        cull_radius = max_track_window + 8.0

        active_ml = cull_points(
            self.ml_arr, car_pos_world.x, car_pos_world.y, cull_radius
        )
        active_ll = cull_points(
            self.ll_arr, car_pos_world.x, car_pos_world.y, cull_radius
        )
        active_rl = cull_points(
            self.rl_arr, car_pos_world.x, car_pos_world.y, cull_radius
        )

        # Track mask extraction: forward cap uses true arc-length along the road
        # centerline. Rear/sides are naturally clipped by camera projection + image bounds.

        car_s = self._get_car_arc_length(car_pos_world.x, car_pos_world.y)
        if car_s is None:
            return

        active_ml_s = self.ml_s[(
            (self.ml_arr[0] - car_pos_world.x) ** 2
            + (self.ml_arr[1] - car_pos_world.y) ** 2
        ) <= cull_radius**2]
        active_ll_s = self.ll_s[(
            (self.ll_arr[0] - car_pos_world.x) ** 2
            + (self.ll_arr[1] - car_pos_world.y) ** 2
        ) <= cull_radius**2]
        active_rl_s = self.rl_s[(
            (self.rl_arr[0] - car_pos_world.x) ** 2
            + (self.rl_arr[1] - car_pos_world.y) ** 2
        ) <= cull_radius**2]

        ml_forward = active_ml_s - car_s
        ll_forward = active_ll_s - car_s
        rl_forward = active_rl_s - car_s

        ml_mask = (ml_forward >= -self.lookbehind_distance) & (
            ml_forward <= self.lookahead_distance
        )
        ll_mask = (ll_forward >= -self.lookbehind_distance) & (
            ll_forward <= self.lookahead_distance
        )
        rl_mask = (rl_forward >= -self.lookbehind_distance) & (
            rl_forward <= (self.lookahead_distance + self.right_lookahead_margin)
        )

        if not np.any(ml_mask):
            return

        cam_ml = np.dot(cam_matrix, active_ml[:, ml_mask])
        cam_ll = np.dot(cam_matrix, active_ll[:, ll_mask])
        cam_rl = np.dot(cam_matrix, active_rl[:, rl_mask])

        t_proj_start = time.perf_counter()

        # Project and map
        valid_center = filter_on_image(self._compute_pixels(cam_ml, w, h), w, h)
        valid_left = filter_on_image(self._compute_pixels(cam_ll, w, h), w, h)
        valid_right = filter_on_image(self._compute_pixels(cam_rl, w, h), w, h)

        t_proj_end = time.perf_counter()

        stamp_str = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
        self._output_results(
            stamp_str, cv_img, valid_left, valid_center, valid_right, msg.header
        )

        t_end = time.perf_counter()

        fps = 1.0 / (t_end - t0)
        self.get_logger().info(
            f"PROF: Total={1000 * (t_end - t0):.1f}ms ({fps:.1f} FPS) | "
            f"TF={1000 * (t_tf - t0):.1f}ms | "
            f"Proj={1000 * (t_proj_end - t_proj_start):.1f}ms | "
            f"Render+Pub={1000 * (t_end - t_proj_end):.1f}ms"
        )

    def _output_results(
        self,
        stamp_str: str,
        img: np.ndarray,
        valid_left: np.ndarray,
        valid_center: np.ndarray,
        valid_right: np.ndarray,
        header,
    ):
        """Saves generated validation states to disk optionally or broadcasts over topics."""
        if self.save_data:
            img_path = os.path.join(self.img_dir, f"{stamp_str}.jpg")
            cv2.imwrite(img_path, img)

            labels = {
                "left": valid_left.astype(int).tolist(),
                "center": valid_center.astype(int).tolist(),
                "right": valid_right.astype(int).tolist(),
            }
            json_path = os.path.join(self.labels_dir, f"{stamp_str}.json")
            with open(json_path, "w") as f:
                json.dump(labels, f)

        debug_img = img.copy()

        draw_poly(debug_img, valid_left, (0, 0, 255))
        draw_poly(debug_img, valid_center, (0, 255, 0))
        draw_poly(debug_img, valid_right, (255, 0, 0))

        if self.save_data:
            debug_path = os.path.join(self.debug_dir, f"{stamp_str}.jpg")
            cv2.imwrite(debug_path, debug_img)

        try:
            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().warning(f"Failed to publish debug image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GroundtruthLabeler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
