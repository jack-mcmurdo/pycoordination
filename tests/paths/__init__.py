"""Thin shim: the path helpers live in :mod:`coordination_oru.util.paths` so
standalone example scripts can use them too. Kept so test bodies keep their
``from tests.paths import ...`` imports.
"""

from coordination_oru.util.paths import (
    line_path,
    load_path,
    load_path_file,
    shuttle_path,
    three_robot_intersection,
    two_robot_cross,
)

__all__ = [
    "line_path",
    "load_path",
    "load_path_file",
    "shuttle_path",
    "three_robot_intersection",
    "two_robot_cross",
]
