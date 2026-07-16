"""Dynamic missions on the bundled demo map, planned by the built-in
Hybrid A* planner.

Three car-like robots live on a 20x20 m occupancy-grid map. In headless or
pyglet mode a scripted scenario runs: the robots swap corners through the
map's corridors (crossing routes the coordinator must sequence), then
drive back to their start poses. With ``--web-viewer`` the example is
**interactive** instead: no scripted missions — click a robot in the
browser, then press-drag-release on the map to post a goal pose (RViz
"2D Nav Goal" style; a plain click aims the goal heading from the robot
toward the click; Esc deselects). Each goal is planned with Hybrid A* and
dispatched as a mission; posting a goal to a driving robot stops it (its
envelope is truncated at the earliest stopping point) and re-tasks it
toward the new goal from wherever it comes to rest.

The implementation lives in :mod:`coordination_oru.demo`, which is also
installed as the ``coordination-oru-demo`` console script (there the web
viewer is the default). This wrapper keeps the examples' historical
default: pyglet/headless unless ``--web-viewer`` is given.

Run:

    python examples/dynamic_missions.py               # scripted (pyglet/headless)
    python examples/dynamic_missions.py --web-viewer  # interactive, point-and-click
"""

from coordination_oru.demo import main

if __name__ == "__main__":
    main(default_viewer="auto")
