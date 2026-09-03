"""Addon preferences: a one-click 'Install Dependencies' button.

`rosbags` is pure-python but not bundled with Blender, so it's pip-installed into a
vendor/ folder inside this addon (added to sys.path in __init__.py) rather than into
Blender's own site-packages, which is read-only under most Blender distributions
(snap/flatpak confinement, system package installs, etc).
"""
import subprocess
import sys
from pathlib import Path

import bpy

VENDOR_DIR = Path(__file__).parent / "vendor"


def dependencies_available() -> bool:
    try:
        import rosbags  # noqa: F401
        return True
    except ImportError:
        return False


class ROSBAG_OT_install_dependencies(bpy.types.Operator):
    """Pip-install rosbags into this addon's bundled vendor/ folder"""
    bl_idname = "rosbag_importer.install_dependencies"
    bl_label = "Install Dependencies"

    def execute(self, context: bpy.types.Context) -> set:
        VENDOR_DIR.mkdir(exist_ok=True)
        try:
            subprocess.run(
                [sys.executable, "-m", "pip", "install", "--target", str(VENDOR_DIR), "rosbags"],
                check=True, capture_output=True, text=True,
            )
        except subprocess.CalledProcessError as e:
            self.report({"ERROR"}, f"pip install failed: {e.stderr[-500:]}")
            return {"CANCELLED"}

        if str(VENDOR_DIR) not in sys.path:
            sys.path.append(str(VENDOR_DIR))
        if not dependencies_available():
            self.report({"ERROR"}, "Install ran but rosbags still not importable")
            return {"CANCELLED"}

        self.report({"INFO"}, "rosbags installed successfully")
        return {"FINISHED"}


class ROSBagImporterPreferences(bpy.types.AddonPreferences):
    bl_idname = __package__

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        if dependencies_available():
            layout.label(text="rosbags is installed.", icon="CHECKMARK")
        else:
            layout.label(text="rosbags (required) is not installed.", icon="ERROR")
            layout.operator(ROSBAG_OT_install_dependencies.bl_idname)


CLASSES = (ROSBAG_OT_install_dependencies, ROSBagImporterPreferences)


def register() -> None:
    for cls in CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
