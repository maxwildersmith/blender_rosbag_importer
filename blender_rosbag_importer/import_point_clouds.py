"""bpy layer: creates keyframed TF empties and native PointCloud objects parented to them.

Each point-cloud message becomes its own PointCloud object (a scan's points generally
differ from the previous scan's, so there's nothing to keyframe on a single object's
point positions) parented to the Empty for its header.frame_id. Only one scan is
visible at a time, via keyframed hide_viewport/hide_render toggles - the parent
Empty's own keyframed transform is what "capturing motion" refers to here.
"""
import logging

import bpy
import mathutils
import numpy as np

from .pointcloud2_decode import decode_pointcloud2
from .tf_tree import TFTree
from .time_alignment import TimeAlignment

logger = logging.getLogger(__name__)


def create_tf_empties(tf_tree: TFTree, align: TimeAlignment, collection: bpy.types.Collection,
                       empty_size: float = 0.05) -> dict[str, bpy.types.Object]:
    """One Empty per TF child frame, keyframed over its recorded samples, parented
    to its own parent frame's Empty where that parent is itself a known child frame.
    """
    empties: dict[str, bpy.types.Object] = {}
    for name, frame in tf_tree.frames.items():
        empty = bpy.data.objects.new(f"tf_{name}", None)
        empty.empty_display_type = "ARROWS"
        empty.empty_display_size = empty_size
        empty.rotation_mode = "QUATERNION"
        collection.objects.link(empty)
        empties[name] = empty

    for name, frame in tf_tree.frames.items():
        empty = empties[name]
        parent = empties.get(frame.parent_frame_id)
        if parent is not None:
            empty.parent = parent

        timestamps_ns, translations, rotations = frame.as_arrays()
        for ts, translation, rotation in zip(timestamps_ns, translations, rotations):
            bpy_frame = align.frame_of(int(ts))
            empty.location = mathutils.Vector(translation)
            # ROS quaternion is (x, y, z, w); Blender's Quaternion() takes (w, x, y, z).
            x, y, z, w = rotation
            empty.rotation_quaternion = mathutils.Quaternion((w, x, y, z))
            empty.keyframe_insert(data_path="location", frame=bpy_frame)
            empty.keyframe_insert(data_path="rotation_quaternion", frame=bpy_frame)

    return empties


def _set_visibility_keyframe(obj: bpy.types.Object, frame: int, visible: bool) -> None:
    obj.hide_viewport = not visible
    obj.hide_render = not visible
    obj.keyframe_insert(data_path="hide_viewport", frame=frame)
    obj.keyframe_insert(data_path="hide_render", frame=frame)


def import_point_cloud_topic(bag_reader, tf_tree: TFTree, topic: str, align: TimeAlignment,
                              empties: dict[str, bpy.types.Object],
                              collection: bpy.types.Collection,
                              max_points_per_scan: int | None = None,
                              stride: int = 1) -> list[bpy.types.Object]:
    """Imports one PointCloud object per message on `topic`, parented to the Empty
    matching each message's header.frame_id, visible only at its own frame.
    """
    created: list[bpy.types.Object] = []
    frame_ids_seen: set[str] = set()

    for i, bag_msg in enumerate(bag_reader.messages(topics=[topic])):
        if i % stride != 0:
            continue
        msg = bag_msg.msg
        frame_id = msg.header.frame_id
        frame_ids_seen.add(frame_id)
        decoded = decode_pointcloud2(msg, max_points=max_points_per_scan)

        name = f"{topic.strip('/').replace('/', '_')}_{i:06d}"
        pc_data = bpy.data.pointclouds.new(name)
        n_points = decoded.xyz.shape[0]
        pc_data.resize(n_points)
        pc_data.attributes["position"].data.foreach_set("vector", decoded.xyz.astype(np.float32).ravel())
        if decoded.rgb is not None:
            rgba = np.ones((n_points, 4), dtype=np.float32)
            rgba[:, :3] = decoded.rgb.astype(np.float32) / 255.0
            color_attr = pc_data.attributes.new("Color", "FLOAT_COLOR", "POINT")
            color_attr.data.foreach_set("color", rgba.ravel())

        obj = bpy.data.objects.new(name, pc_data)
        collection.objects.link(obj)
        created.append(obj)

        parent = empties.get(frame_id)
        if parent is not None:
            obj.parent = parent
        else:
            logger.warning("Point cloud message on %s references unknown frame_id '%s'",
                            topic, frame_id)

        bpy_frame = align.frame_of(bag_msg.timestamp_ns)
        _set_visibility_keyframe(obj, bpy_frame - 1, visible=False)
        _set_visibility_keyframe(obj, bpy_frame, visible=True)
        _set_visibility_keyframe(obj, bpy_frame + 1, visible=False)

    if len(frame_ids_seen) > 1:
        logger.warning("Topic %s published under multiple frame_ids: %s", topic, frame_ids_seen)

    return created
