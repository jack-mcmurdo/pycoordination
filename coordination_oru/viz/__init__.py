"""Optional visualisation front-ends (``pip install coordination-oru[viz]``).

Imports are lazy (PEP 562) so that each viewer only requires its own
dependencies: :class:`PygletViewer` needs pyglet, :class:`WebViewer` needs
starlette + uvicorn.
"""

from typing import Any

__all__ = ["PygletViewer", "WebViewer"]


def __getattr__(name: str) -> Any:
    if name == "PygletViewer":
        from coordination_oru.viz.pyglet_viewer import PygletViewer

        return PygletViewer
    if name == "WebViewer":
        from coordination_oru.viz.web_viewer import WebViewer

        return WebViewer
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
