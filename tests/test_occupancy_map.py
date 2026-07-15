"""OccupancyMap: PGM/YAML loading, transforms, inflation, PNG export."""

from __future__ import annotations

import pathlib

import numpy as np
import pytest

from coordination_oru.motionplanning import OccupancyMap, load_bundled_map

WIDTH, HEIGHT = 8, 10


def _write_map(
    tmp_path: pathlib.Path, *, negate: int = 0, mode: str | None = None, yaw: float = 0.0
) -> pathlib.Path:
    """A 10x8 P5 PGM (8 wide, 10 tall), all 254 except a 2x2 block of 0 at
    the image top-left, plus its YAML descriptor."""
    img = np.full((HEIGHT, WIDTH), 254, dtype=np.uint8)
    img[0:2, 0:2] = 0
    (tmp_path / "tiny.pgm").write_bytes(
        f"P5\n# a comment\n{WIDTH} {HEIGHT}\n255\n".encode() + img.tobytes()
    )
    yaml_path = tmp_path / "tiny.yaml"
    lines = [
        "image: tiny.pgm",
        "resolution: 0.5",
        f"origin: [1.0, 2.0, {yaw}]",
        f"negate: {negate}",
    ]
    if mode is not None:
        lines.append(f"mode: {mode}")
    yaml_path.write_text("\n".join(lines) + "\n")
    return yaml_path


def test_loader_shape_and_bounds(tmp_path: pathlib.Path) -> None:
    m = OccupancyMap.from_yaml(_write_map(tmp_path))
    assert m.width == 8
    assert m.height == 10
    assert m.bounds == (1.0, 2.0, 5.0, 7.0)


def test_y_flip(tmp_path: pathlib.Path) -> None:
    m = OccupancyMap.from_yaml(_write_map(tmp_path))
    # the occupied block is image-top-left => world-top-left
    assert m.occupied[9, 0]
    assert m.occupied[8, 1]
    assert not m.occupied[0, 0]


def test_transform_roundtrip(tmp_path: pathlib.Path) -> None:
    m = OccupancyMap.from_yaml(_write_map(tmp_path))
    for row, col in [(0, 0), (9, 7), (5, 4)]:
        assert m.world_to_grid(*m.grid_to_world(row, col)) == (row, col)


def test_negate_and_unknown(tmp_path: pathlib.Path) -> None:
    negated = OccupancyMap.from_yaml(_write_map(tmp_path, negate=1))
    # negate flips which pixels are occupied: the 254 background is now dark
    assert negated.occupied[0, 5]
    assert not negated.occupied[9, 0]

    gray = np.full((HEIGHT, WIDTH), 254, dtype=np.uint8)
    gray[4, 4] = 205  # ROS "unknown" gray
    (tmp_path / "gray.pgm").write_bytes(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode() + gray.tobytes())
    (tmp_path / "gray.yaml").write_text(
        "image: gray.pgm\nresolution: 0.5\norigin: [0.0, 0.0, 0.0]\n"
    )
    strict = OccupancyMap.from_yaml(tmp_path / "gray.yaml")
    lenient = OccupancyMap.from_yaml(tmp_path / "gray.yaml", unknown_is_occupied=False)
    assert strict.occupied[HEIGHT - 1 - 4, 4]
    assert not lenient.occupied[HEIGHT - 1 - 4, 4]


def test_inflation(tmp_path: pathlib.Path) -> None:
    m = OccupancyMap.from_yaml(_write_map(tmp_path))
    inflated = m.inflated(0.5)
    assert inflated.sum() > m.occupied.sum()
    assert bool(np.all(inflated[m.occupied]))
    assert bool(np.array_equal(m.inflated(0.0), m.occupied))


def test_png_export(tmp_path: pathlib.Path) -> None:
    m = OccupancyMap.from_yaml(_write_map(tmp_path))
    png = m.to_png_bytes()
    assert png.startswith(b"\x89PNG\r\n\x1a\n")
    assert b"IEND" in png


def test_rejects_rotated_and_raw(tmp_path: pathlib.Path) -> None:
    with pytest.raises(ValueError):
        OccupancyMap.from_yaml(_write_map(tmp_path, yaw=1.57))
    with pytest.raises(ValueError):
        OccupancyMap.from_yaml(_write_map(tmp_path, mode="raw"))


def test_load_bundled_map() -> None:
    m = load_bundled_map()
    assert m.width == 400
    assert m.height == 400
