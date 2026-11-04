"""Caching helpers for rendered road textures."""

import hashlib
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Protocol


_CACHE_VERSION = 1


class RenderSize(Protocol):
    """Dimensions returned by a road renderer."""

    x: float
    y: float


class RoadRenderer(Protocol):
    """Road data required by the render cache."""

    _name: str
    use_seed: bool

    def render_to_file(self, path: str) -> RenderSize:
        """Render the road image into a directory."""


@dataclass(frozen=True)
class CachedRenderSize:
    """Two-dimensional road image size restored from the cache manifest."""

    x: float
    y: float


def _render_inputs_fingerprint(
    package_source: Path,
    road_source: Path,
    seed: str,
) -> str:
    """Hash Python sources and the seed that determine rendered road output."""
    digest = hashlib.sha256()
    digest.update(f"cache-version:{_CACHE_VERSION}\nseed:{seed}\n".encode())

    for label, root in (("package", package_source), ("road", road_source)):
        if not root.is_dir():
            raise FileNotFoundError(f"Render source directory does not exist: {root}")
        for source_file in sorted(root.rglob("*.py")):
            relative_path = source_file.relative_to(root).as_posix()
            digest.update(f"{label}/{relative_path}\n".encode())
            digest.update(source_file.read_bytes())

    return digest.hexdigest()


def render_to_file_cached(
    road: RoadRenderer,
    output_directory: str,
    package_source: str,
    road_source: str,
    seed: str,
) -> tuple[RenderSize, bool]:
    """Render a road only when its source or generated image has changed."""
    output_path = Path(output_directory)
    output_path.mkdir(parents=True, exist_ok=True)

    image_path = output_path / f"{road._name}.png"
    manifest_path = output_path / f".{road._name}.render-cache.json"
    if not road.use_seed:
        manifest_path.unlink(missing_ok=True)
        size = road.render_to_file(str(output_path))
        return size, False

    fingerprint = _render_inputs_fingerprint(
        Path(package_source),
        Path(road_source),
        seed,
    )

    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        manifest = None
    except (json.JSONDecodeError, UnicodeDecodeError):
        manifest = None

    if isinstance(manifest, dict):
        image_size = manifest.get("image_size_bytes")
        try:
            image_stat = image_path.stat()
        except FileNotFoundError:
            image_stat = None

        if (
            manifest.get("cache_version") == _CACHE_VERSION
            and manifest.get("fingerprint") == fingerprint
            and isinstance(image_size, int)
            and not isinstance(image_size, bool)
            and image_size > 0
            and image_stat is not None
            and image_path.is_file()
            and image_stat.st_size == image_size
        ):
            dimensions = manifest.get("dimensions")
            if (
                isinstance(dimensions, list)
                and len(dimensions) == 2
                and all(
                    isinstance(value, (int, float))
                    and not isinstance(value, bool)
                    and math.isfinite(value)
                    and value > 0
                    for value in dimensions
                )
            ):
                return CachedRenderSize(float(dimensions[0]), float(dimensions[1])), True

    manifest_path.unlink(missing_ok=True)
    size = road.render_to_file(str(output_path))
    if not image_path.is_file() or image_path.stat().st_size == 0:
        raise RuntimeError(f"Road renderer did not produce {image_path}")

    manifest_data = {
        "cache_version": _CACHE_VERSION,
        "fingerprint": fingerprint,
        "dimensions": [size.x, size.y],
        "image_size_bytes": image_path.stat().st_size,
    }

    temporary_path = None
    try:
        with NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=output_path,
            prefix=f".{road._name}.render-cache-",
            suffix=".tmp",
            delete=False,
        ) as temporary_file:
            temporary_path = Path(temporary_file.name)
            json.dump(manifest_data, temporary_file)
        os.replace(temporary_path, manifest_path)
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)

    return size, False
