bl_info = {
    "name": "ROS Bag Importer",
    "description": "Import point clouds, TF motion, and image sequences from ROS bags",
    "author": "Max Wilder-Smith",
    "version": (0, 1, 0),
    "blender": (4, 2, 0),
    "location": "File > Import > ROS Bag",
    "category": "Import-Export",
}

import sys
from pathlib import Path

_VENDOR_DIR = Path(__file__).parent / "vendor"
if _VENDOR_DIR.exists() and str(_VENDOR_DIR) not in sys.path:
    sys.path.append(str(_VENDOR_DIR))

from . import preferences

_DEPENDENCIES_AVAILABLE = preferences.dependencies_available()

if _DEPENDENCIES_AVAILABLE:
    from . import operators, properties, ui_list
    _MODULES = (properties, ui_list, operators)
else:
    _MODULES = ()


def register() -> None:
    preferences.register()
    for module in _MODULES:
        module.register()
    if not _DEPENDENCIES_AVAILABLE:
        print("[rosbag_importer] rosbags not installed - install it from "
              "Edit > Preferences > Add-ons > ROS Bag Importer")


def unregister() -> None:
    for module in reversed(_MODULES):
        module.unregister()
    preferences.unregister()


if __name__ == "__main__":
    register()
