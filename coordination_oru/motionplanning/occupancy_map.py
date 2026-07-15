"""ROS-style occupancy-grid maps (YAML + PGM) for motion planning.

``OccupancyMap`` loads a `map_server`-style YAML descriptor plus its image,
stores the grid **y-up** (row index increases with world +y), converts
between world and grid coordinates, inflates obstacles by a robot radius
(cached per radius), and exports a PNG for the web viewer.
"""

from __future__ import annotations

import importlib.resources
import math
import pathlib
import struct
import zlib

import numpy as np
import numpy.typing as npt
import scipy.ndimage
import yaml

__all__ = ["OccupancyMap", "load_bundled_map"]


def _read_pgm(path: pathlib.Path) -> npt.NDArray[np.uint8]:
    """Read a binary (P5) or ASCII (P2) PGM file as a ``(height, width)``
    ``uint8`` array. ``#`` comments in the header are honored."""
    data = path.read_bytes()
    tokens: list[bytes] = []
    pos = 0
    n = len(data)
    while len(tokens) < 4 and pos < n:
        byte = data[pos : pos + 1]
        if byte.isspace():
            pos += 1
            continue
        if byte == b"#":
            while pos < n and data[pos : pos + 1] not in (b"\n", b"\r"):
                pos += 1
            continue
        start = pos
        while pos < n and not data[pos : pos + 1].isspace() and data[pos : pos + 1] != b"#":
            pos += 1
        tokens.append(data[start:pos])
    if len(tokens) < 4:
        raise ValueError(f"truncated PGM header in {path}")
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic not in (b"P5", b"P2"):
        raise ValueError(f"unsupported PGM magic {magic!r} in {path} (want P5 or P2)")
    if maxval > 255:
        raise ValueError(f"unsupported PGM maxval {maxval} in {path} (want <= 255)")
    if magic == b"P5":
        # pixel data starts after the single whitespace byte following maxval
        raw = data[pos + 1 : pos + 1 + width * height]
        if len(raw) < width * height:
            raise ValueError(f"truncated PGM pixel data in {path}")
        return np.frombuffer(raw, dtype=np.uint8).reshape(height, width)
    ascii_tokens = b" ".join(
        line.split(b"#", 1)[0] for line in data[pos:].splitlines()
    ).split()
    return np.array([int(t) for t in ascii_tokens], dtype=np.uint8).reshape(height, width)


class OccupancyMap:
    """A y-up occupancy grid with world<->grid transforms and inflation."""

    def __init__(
        self,
        image: npt.NDArray[np.uint8],
        resolution: float,
        origin: tuple[float, float],
        occupied: npt.NDArray[np.bool_],
    ) -> None:
        self.image = image  # (height, width), already flipped to y-up
        self.resolution = resolution  # metres/pixel
        self.origin = origin  # world (x, y) of the map's lower-left corner
        self.occupied = occupied
        self._inflated_cache: dict[int, npt.NDArray[np.bool_]] = {}

    @classmethod
    def from_yaml(
        cls, yaml_path: str | pathlib.Path, *, unknown_is_occupied: bool = True
    ) -> "OccupancyMap":
        """Load a ROS `map_server` YAML descriptor and its image.

        Cells with occupancy probability above ``occupied_thresh`` are
        occupied; the unknown band between ``free_thresh`` and
        ``occupied_thresh`` counts as occupied unless
        ``unknown_is_occupied=False``. Only ``trinary`` mode and unrotated
        maps (``origin`` yaw 0) are supported.
        """
        yaml_path = pathlib.Path(yaml_path)
        with yaml_path.open() as f:
            spec = yaml.safe_load(f)
        for key in ("image", "resolution", "origin"):
            if key not in spec:
                raise ValueError(f"map YAML {yaml_path} is missing required key {key!r}")
        mode = spec.get("mode", "trinary")
        if mode != "trinary":
            raise ValueError(f"unsupported map mode {mode!r} (only 'trinary' is supported)")
        origin = spec["origin"]
        if len(origin) > 2 and float(origin[2]) != 0.0:
            raise ValueError("rotated maps (nonzero origin yaw) are unsupported")
        negate = int(spec.get("negate", 0))
        occupied_thresh = float(spec.get("occupied_thresh", 0.65))
        free_thresh = float(spec.get("free_thresh", 0.196))

        image_path = yaml_path.parent / str(spec["image"])
        if image_path.suffix.lower() == ".pgm":
            raw = _read_pgm(image_path)
        else:
            try:
                from PIL import Image
            except ImportError:
                raise ValueError("non-PGM maps need Pillow: pip install pillow") from None
            raw = np.asarray(Image.open(image_path).convert("L"), dtype=np.uint8)
        img = np.flipud(raw).copy()

        p = img / 255.0 if negate else (255 - img) / 255.0
        occupied = p > occupied_thresh
        if unknown_is_occupied:
            occupied |= p > free_thresh
        return cls(img, float(spec["resolution"]), (float(origin[0]), float(origin[1])), occupied)

    @property
    def height(self) -> int:
        return int(self.image.shape[0])

    @property
    def width(self) -> int:
        return int(self.image.shape[1])

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        """World-frame ``(xmin, ymin, xmax, ymax)`` of the map."""
        ox, oy = self.origin
        return (ox, oy, ox + self.width * self.resolution, oy + self.height * self.resolution)

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """The ``(row, col)`` cell containing world point ``(x, y)``."""
        return (
            int(math.floor((y - self.origin[1]) / self.resolution)),
            int(math.floor((x - self.origin[0]) / self.resolution)),
        )

    def grid_to_world(self, row: int, col: int) -> tuple[float, float]:
        """The world ``(x, y)`` of the center of cell ``(row, col)``."""
        return (
            self.origin[0] + (col + 0.5) * self.resolution,
            self.origin[1] + (row + 0.5) * self.resolution,
        )

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def inflated(self, radius: float) -> npt.NDArray[np.bool_]:
        """The occupancy grid dilated by ``radius`` metres (a disk
        structuring element). Cached per pixel radius; callers must
        ``.copy()`` before writing."""
        k = math.ceil(radius / self.resolution)
        cached = self._inflated_cache.get(k)
        if cached is not None:
            return cached
        if k == 0:
            result = self.occupied
        else:
            yy, xx = np.mgrid[-k : k + 1, -k : k + 1]
            disk = (xx * xx + yy * yy) <= k * k
            result = scipy.ndimage.binary_dilation(self.occupied, structure=disk)
        self._inflated_cache[k] = result
        return result

    def to_png_bytes(self) -> bytes:
        """The map as a grayscale 8-bit PNG (image orientation, top row
        first), encoded with the stdlib only."""

        def _chunk(tag: bytes, data: bytes) -> bytes:
            return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data))

        img = np.flipud(self.image)
        h, w = img.shape
        raw = b"".join(b"\x00" + img[r].tobytes() for r in range(h))
        ihdr = struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)  # bit depth 8, color type 0 (gray)
        return (
            b"\x89PNG\r\n\x1a\n"
            + _chunk(b"IHDR", ihdr)
            + _chunk(b"IDAT", zlib.compress(raw))
            + _chunk(b"IEND", b"")
        )


def load_bundled_map(name: str = "demo.yaml") -> OccupancyMap:
    """Load a map bundled as package data under ``coordination_oru/data/maps/``."""
    path = pathlib.Path(str(importlib.resources.files("coordination_oru.data") / "maps" / name))
    return OccupancyMap.from_yaml(path)
