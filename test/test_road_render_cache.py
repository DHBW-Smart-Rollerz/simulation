import json
from pathlib import Path
from types import SimpleNamespace

from simulation.groundtruth.road.render_cache import render_to_file_cached


class FakeRoad:
    _name = "test_road"
    use_seed = True

    def __init__(self):
        self.render_count = 0

    def render_to_file(self, path):
        self.render_count += 1
        Path(path, f"{self._name}.png").write_bytes(b"rendered road")
        return SimpleNamespace(x=10.0, y=20.0)


def test_cached_render_reuses_image_for_unchanged_sources(tmp_path):
    package_source = tmp_path / "package"
    road_source = tmp_path / "roads" / "test_road"
    package_source.mkdir()
    road_source.mkdir(parents=True)
    (package_source / "renderer.py").write_text("version = 1\n", encoding="utf-8")
    (road_source / "road.py").write_text("road_input = 1\n", encoding="utf-8")
    road = FakeRoad()

    first_size, first_cache_hit = render_to_file_cached(
        road, str(road_source), str(package_source), str(road_source), "seed"
    )
    second_size, second_cache_hit = render_to_file_cached(
        road, str(road_source), str(package_source), str(road_source), "seed"
    )

    assert (first_size.x, first_size.y) == (10.0, 20.0)
    assert (second_size.x, second_size.y) == (10.0, 20.0)
    assert first_cache_hit is False
    assert second_cache_hit is True
    assert road.render_count == 1


def test_cached_render_rerenders_when_road_source_changes(tmp_path):
    package_source = tmp_path / "package"
    road_source = tmp_path / "roads" / "test_road"
    package_source.mkdir()
    road_source.mkdir(parents=True)
    (package_source / "renderer.py").write_text("version = 1\n", encoding="utf-8")
    road_definition = road_source / "road.py"
    road_definition.write_text("road_input = 1\n", encoding="utf-8")
    road = FakeRoad()

    render_to_file_cached(
        road, str(road_source), str(package_source), str(road_source), "seed"
    )
    road_definition.write_text("road_input = 2\n", encoding="utf-8")
    _, cache_hit = render_to_file_cached(
        road, str(road_source), str(package_source), str(road_source), "seed"
    )

    assert cache_hit is False
    assert road.render_count == 2


def test_cached_render_rerenders_when_image_is_missing(tmp_path):
    package_source = tmp_path / "package"
    road_source = tmp_path / "roads" / "test_road"
    package_source.mkdir()
    road_source.mkdir(parents=True)
    (package_source / "renderer.py").write_text("version = 1\n", encoding="utf-8")
    (road_source / "road.py").write_text("road_input = 1\n", encoding="utf-8")
    road = FakeRoad()

    render_to_file_cached(
        road, str(road_source), str(package_source), str(road_source), "seed"
    )
    (road_source / f"{road._name}.png").unlink()
    _, cache_hit = render_to_file_cached(
        road, str(road_source), str(package_source), str(road_source), "seed"
    )

    assert cache_hit is False
    assert road.render_count == 2


def test_invalid_cache_manifest_is_rebuilt(tmp_path):
    package_source = tmp_path / "package"
    road_source = tmp_path / "roads" / "test_road"
    package_source.mkdir()
    road_source.mkdir(parents=True)
    (package_source / "renderer.py").write_text("version = 1\n", encoding="utf-8")
    (road_source / "road.py").write_text("road_input = 1\n", encoding="utf-8")
    (road_source / ".test_road.render-cache.json").write_text(
        json.dumps({"cache_version": 1}),
        encoding="utf-8",
    )
    road = FakeRoad()

    _, cache_hit = render_to_file_cached(
        road, str(road_source), str(package_source), str(road_source), "seed"
    )

    assert cache_hit is False
    assert road.render_count == 1


def test_unseeded_road_is_rendered_every_time(tmp_path):
    package_source = tmp_path / "package"
    road_source = tmp_path / "roads" / "test_road"
    package_source.mkdir()
    road_source.mkdir(parents=True)
    (package_source / "renderer.py").write_text("version = 1\n", encoding="utf-8")
    (road_source / "road.py").write_text("road_input = 1\n", encoding="utf-8")
    road = FakeRoad()
    road.use_seed = False

    _, first_cache_hit = render_to_file_cached(
        road, str(road_source), str(package_source), str(road_source), "seed"
    )
    _, second_cache_hit = render_to_file_cached(
        road, str(road_source), str(package_source), str(road_source), "seed"
    )

    assert first_cache_hit is False
    assert second_cache_hit is False
    assert road.render_count == 2
