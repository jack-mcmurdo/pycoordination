"""Deterministic generator for the bundled demo map.

Writes ``coordination_oru/data/maps/demo.pgm`` (400x400 px binary PGM,
0.05 m/px => 20x20 m, origin at (-10, -10)) and its ``demo.yaml``
descriptor. Obstacles: 0.2 m border walls, two square blocks and an
L-shaped wall; all corridors stay >= 3.5 m wide.
"""

from __future__ import annotations

import pathlib

import numpy as np

FREE = 254
OCCUPIED = 0
SIZE = 400
RESOLUTION = 0.05
ORIGIN = (-10.0, -10.0)

OUT_DIR = pathlib.Path(__file__).resolve().parent.parent / "coordination_oru" / "data" / "maps"


def main() -> None:
    # y-up array: row index increases with world +y
    img = np.full((SIZE, SIZE), FREE, dtype=np.uint8)
    xs = ORIGIN[0] + (np.arange(SIZE) + 0.5) * RESOLUTION  # cell-center world coords
    ys = ORIGIN[1] + (np.arange(SIZE) + 0.5) * RESOLUTION

    def fill(x0: float, y0: float, x1: float, y1: float) -> None:
        """Mark occupied every pixel whose cell center lies inside the world rect."""
        cols = (xs >= x0) & (xs <= x1)
        rows = (ys >= y0) & (ys <= y1)
        img[np.ix_(rows, cols)] = OCCUPIED

    # border walls, 0.2 m thick along all four edges
    fill(-10.0, -10.0, 10.0, -9.8)
    fill(-10.0, 9.8, 10.0, 10.0)
    fill(-10.0, -10.0, -9.8, 10.0)
    fill(9.8, -10.0, 10.0, 10.0)
    # blocks
    fill(-6.0, -2.0, -2.0, 2.0)
    fill(2.0, -6.0, 6.0, -2.0)
    # L-wall
    fill(-2.0, 4.0, 6.0, 4.4)
    fill(5.6, 0.0, 6.0, 4.4)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    # PGM row 0 is the map's top: flip the y-up array back to image order
    pgm = np.flipud(img)
    (OUT_DIR / "demo.pgm").write_bytes(b"P5\n400 400\n255\n" + pgm.tobytes())
    (OUT_DIR / "demo.yaml").write_text(
        "image: demo.pgm\n"
        "resolution: 0.05\n"
        "origin: [-10.0, -10.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    print(f"wrote {OUT_DIR / 'demo.pgm'} and {OUT_DIR / 'demo.yaml'}")


if __name__ == "__main__":
    main()
