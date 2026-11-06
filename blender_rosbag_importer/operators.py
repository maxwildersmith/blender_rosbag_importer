"""IMPORT_OT_rosbag: the main import operator.

Standard Blender ImportHelper pattern: invoke() opens a file browser, draw() renders
a per-topic checklist (populated by scanning the chosen bag on each filepath change),
execute() runs the actual import once the user confirms.
"""
import logging
from pathlib import Path

import bpy
from bpy_extras.io_utils import ImportHelper

from . import (grid_map_cache, import_grid_map, import_images, import_point_clouds,
               materials, point_cloud_cache)
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
    grid_map_stride: bpy.props.IntProperty(
        name="Grid Map Stride", default=1, min=1,
        description="Import every Nth grid map message",
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
        col.prop(self, "grid_map_stride")

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
        point_cloud_material = None
        grid_map_material = None

        for topic in selected:
            if topic.kind == "POINTCLOUD":
                cache_dir = frames_dir / f"{topic.name.strip('/').replace('/', '_')}_cache"
                cache = point_cloud_cache.cache_point_cloud_topic(
                    reader, topic.name, cache_dir, align,
                    max_points_per_scan=max_points, stride=self.point_stride,
                )
                if point_cloud_material is None:
                    point_cloud_material = materials.ensure_point_cloud_material()
                import_point_clouds.create_animated_point_cloud(
                    topic.name, cache, empties, collection, point_cloud_material,
                )
                self.report({"INFO"}, f"{topic.name}: {len(cache.frame_to_path)} point cloud scans (animated)")
            elif topic.kind == "GRIDMAP":
                cache_dir = frames_dir / f"{topic.name.strip('/').replace('/', '_')}_cache"
                cache = grid_map_cache.cache_grid_map_topic(
                    reader, topic.name, cache_dir, align, stride=self.grid_map_stride,
                )
                if grid_map_material is None:
                    grid_map_material = materials.ensure_grid_map_material()
                import_grid_map.create_animated_grid_map(
                    topic.name, cache, empties, collection, grid_map_material,
                )
                self.report({"INFO"}, f"{topic.name}: {len(cache.frame_to_path)} grid map scans (animated)")
            elif topic.kind == "IMAGE":
                out_dir = frames_dir / topic.name.strip("/").replace("/", "_")
                sequence = import_images.extract_image_sequence(
                    reader, topic.name, out_dir, align, stride=self.image_stride,
                )
                image = import_images.load_image_sequence(sequence, name=topic.name.strip("/"))
                if image is not None:
                    empty = import_images.create_image_empty(
                        image, sequence, collection, name=f"img_{topic.name.strip('/')}",
                    )
                    # Best-effort: parent to the TF frame matching the topic's camera,
                    # if a frame with a matching name prefix was published.
                    camera_frame = _guess_camera_frame(topic.name, empties)
                    if camera_frame is not None:
                        empty.parent = empties[camera_frame]
                self.report({"INFO"}, f"{topic.name}: {len(sequence.paths)} image frames (animated)")

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
