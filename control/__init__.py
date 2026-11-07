"""Local control package.

The installed python-control library uses this same top-level name. Loading
this folder first would hide that library, and the LQR gain comes from it.
This module loads the installed library, then attaches halo_controls to it.
"""

import importlib
import importlib.util
import sys
from pathlib import Path

_here = Path(__file__).resolve().parent
_repo = _here.parent


def _load_installed_control():
    sys.modules.pop("control", None)
    for key in list(sys.modules):
        if key.startswith("control."):
            sys.modules.pop(key)
    kept = []
    hidden = []
    for entry in sys.path:
        try:
            if Path(entry or ".").resolve() == _repo:
                hidden.append(entry)
                continue
        except OSError:
            pass
        kept.append(entry)
    sys.path[:] = kept
    library = importlib.import_module("control")
    for entry in reversed(hidden):
        sys.path.insert(0, entry)
    return library


_library = _load_installed_control()
_spec = importlib.util.spec_from_file_location(
    "control.halo_controls",
    _here / "halo_controls.py",
)
_halo = importlib.util.module_from_spec(_spec)
sys.modules["control.halo_controls"] = _halo
_spec.loader.exec_module(_halo)
_library.halo_controls = _halo
