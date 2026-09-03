"""RosbagTopicEntry: one row of the per-file topic checklist shown by the import
operator (blender_rosbag_importer.operators.IMPORT_OT_rosbag). A PropertyGroup
rather than flat Scene attributes (unlike articulated_splats/blender_plugin.py's
INPUT_PROPERTIES list) since the topic list is dynamic - populated fresh each time
the user picks a bag file - and lives on the operator itself, not the Scene, so it
never goes stale between imports of different bags.
"""
import bpy

TOPIC_KIND_ITEMS = [
    ("POINTCLOUD", "Point Cloud", "Import as native Blender point cloud objects"),
    ("IMAGE", "Image Sequence", "Import as a Blender image sequence"),
    ("SKIP", "Skip", "Do not import this topic"),
]

_MSGTYPE_TO_DEFAULT_KIND = {
    "sensor_msgs/msg/PointCloud2": "POINTCLOUD",
    "sensor_msgs/msg/Image": "IMAGE",
    "sensor_msgs/msg/CompressedImage": "IMAGE",
}


class RosbagTopicEntry(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(name="Topic")
    msgtype: bpy.props.StringProperty(name="Type")
    count: bpy.props.IntProperty(name="Messages")
    selected: bpy.props.BoolProperty(name="Import", default=False)
    kind: bpy.props.EnumProperty(name="Import As", items=TOPIC_KIND_ITEMS, default="SKIP")


def default_kind_for(msgtype: str) -> str:
    return _MSGTYPE_TO_DEFAULT_KIND.get(msgtype, "SKIP")


CLASSES = (RosbagTopicEntry,)


def register() -> None:
    for cls in CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
