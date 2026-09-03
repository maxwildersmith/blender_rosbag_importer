"""IMPORT_OT_rosbag: the main import operator.

Standard Blender ImportHelper pattern: invoke() opens a file browser, draw() renders
a per-topic checklist (populated by scanning the chosen bag on each filepath change),
execute() runs the actual import once the user confirms.
"""
import logging
from pathlib import Path

import bpy
from bpy_extras.io_utils import ImportHelper

from . import import_images, import_point_clouds
from .bag_reader import BagReader
from .properties import RosbagTopicEntry, default_kind_for
from .tf_tree import TFTree
from .time_alignment import TimeAlignment
from .ui_list import ROSBAG_UL_topics

logger = logging.getLogger(__name__)


def _scan_topics(operator: "IMPORT_OT_rosbag", bag_path: Path) -> None:
    operator.topics.clear()
    try:
        topics = BagReader(bag_path).topics()
    except Exception as e:
        logger.warning("Could not scan topics from %s: %s", bag_path, e)
        return
    for name, info in sorted(topics.items()):
        entry = operator.topics.add()
        entry.name = name
        entry.msgtype = info.msgtype
        entry.count = info.count
        entry.kind = default_kind_for(info.msgtype)
        entry.selected = entry.kind != "SKIP"


def _on_filepath_update(operator: "IMPORT_OT_rosbag", context: bpy.types.Context) -> None:
    path = Path(operator.filepath)
    if path.exists() and path.is_file():
        _scan_topics(operator, path)


class IMPORT_OT_rosbag(bpy.types.Operator, ImportHelper):
    """Import point clouds, TF motion, and image sequences from a ROS bag"""
    bl_idname = "import_scene.rosbag"
    bl_label = "Import ROS Bag"
    bl_options = {"UNDO"}

    filename_ext = ".bag"
    filter_glob: bpy.props.StringProperty(default="*.bag;*.db3;*.mcap", options={"HIDDEN"})
    filepath: bpy.props.StringProperty(subtype="FILE_PATH", update=_on_filepath_update)

    fps: bpy.props.FloatProperty(
        name="FPS", default=30.0, min=1.0,
        description="Blender frame rate used to convert bag timestamps to frame numbers",
    )
    point_stride: bpy.props.IntProperty(
        name="Point Cloud Stride", default=1, min=1,
        description="Import every Nth point-cloud message",
    )
    max_points_per_scan: bpy.props.IntProperty(
        name="Max Points/Scan", default=200000, min=0,
        description="Decimate each point cloud message to roughly this many points (0 = no limit)",
    )
    image_stride: bpy.props.IntProperty(
        name="Image Stride", default=1, min=1,
        description="Import every Nth image message",
    )
    topics: bpy.props.CollectionProperty(type=RosbagTopicEntry)
    topics_active_index: bpy.props.IntProperty()

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        layout.prop(self, "fps")
        col = layout.column(align=True)
        col.prop(self, "point_stride")
        col.prop(self, "max_points_per_scan")
        col.prop(self, "image_stride")

        layout.label(text="Topics:")
        if not self.topics:
            layout.label(text="(choose a bag file to see its topics)", icon="INFO")
            return
        layout.template_list("ROSBAG_UL_topics", "", self, "topics",
                              self, "topics_active_index", rows=min(8, len(self.topics)))

    def execute(self, context: bpy.types.Context) -> set:
        bag_path = Path(self.filepath)
        if not bag_path.exists():
            self.report({"ERROR"}, f"{bag_path} does not exist")
            return {"CANCELLED"}
        if not self.topics:
            _scan_topics(self, bag_path)

        selected = [t for t in self.topics if t.selected and t.kind != "SKIP"]
        if not selected:
            self.report({"ERROR"}, "No topics selected for import")
            return {"CANCELLED"}

        reader = BagReader(bag_path)
        selected_names = [t.name for t in selected]
        try:
            start_time_ns = reader.start_time_ns(selected_names)
        except ValueError as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}
        align = TimeAlignment.starting_at_zero(start_time_ns, fps=self.fps)
        end_time_ns = reader.end_time_ns(selected_names)
        context.scene.frame_start = 0
        context.scene.frame_end = max(1, align.frame_of(end_time_ns))
        context.scene.render.fps = round(self.fps)

        collection = bpy.data.collections.new(bag_path.stem)
        context.scene.collection.children.link(collection)

        tf_tree = TFTree.from_bag(reader)
        empties = import_point_clouds.create_tf_empties(tf_tree, align, collection)
        self.report({"INFO"}, f"Created {len(empties)} TF empties")

        max_points = self.max_points_per_scan or None
        frames_dir = bag_path.parent / f"{bag_path.stem}_frames"

        for topic in selected:
            if topic.kind == "POINTCLOUD":
                created = import_point_clouds.import_point_cloud_topic(
                    reader, tf_tree, topic.name, align, empties, collection,
                    max_points_per_scan=max_points, stride=self.point_stride,
                )
                self.report({"INFO"}, f"{topic.name}: {len(created)} point cloud scans")
            elif topic.kind == "IMAGE":
                out_dir = frames_dir / topic.name.strip("/").replace("/", "_")
                paths = import_images.extract_image_sequence(
                    reader, topic.name, out_dir, align, stride=self.image_stride,
                )
                image = import_images.load_image_sequence(paths, name=topic.name.strip("/"))
                if image is not None:
                    empty = import_images.create_image_empty(
                        image, len(paths), collection, name=f"img_{topic.name.strip('/')}",
                    )
                    # Best-effort: parent to the TF frame matching the topic's camera,
                    # if a frame with a matching name prefix was published.
                    camera_frame = _guess_camera_frame(topic.name, empties)
                    if camera_frame is not None:
                        empty.parent = empties[camera_frame]
                self.report({"INFO"}, f"{topic.name}: {len(paths)} image frames")

        return {"FINISHED"}


def _guess_camera_frame(topic: str, empties: dict) -> str | None:
    prefix = topic.strip("/").split("/")[0]
    for name in empties:
        if name.startswith(prefix):
            return name
    return None


CLASSES = (IMPORT_OT_rosbag,)


def menu_func_import(self, context: bpy.types.Context) -> None:
    self.layout.operator(IMPORT_OT_rosbag.bl_idname, text="ROS Bag (.bag/.db3/.mcap)")


def register() -> None:
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)


def unregister() -> None:
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
