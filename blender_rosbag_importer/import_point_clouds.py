"""bpy layer: creates keyframed TF empties, and one animated Point Cloud object per
topic whose data is updated in place as the timeline scrubs (via a frame_change_pre
handler reading point_cloud_cache.py's per-scan .npz cache) rather than a stack of
hidden objects toggled on/off - a real per-point-count-varying animated point cloud
has no single mesh to keyframe with Blender's ordinary fcurve system, and per-point
fcurves don't scale past a handful of points, so this is the practical approach for
real point-cloud scale.

Rather than snapping straight to a new scan's data the instant its frame is
crossed, the object always holds BOTH the outgoing and incoming scan's points at
once (tagged via `_pc_src`/`_pc_rand` attributes), and point_cloud_nodes.py's shared
Geometry Nodes modifier crossfades between them - see that module's docstring for
why (no per-point correspondence between scans) and what tradeoff that costs
(point count roughly doubles for the span between any two scans' frames).

Which scan(s) bracket the current frame - and holding the first scan before its
frame and the last scan forever after its frame, rather than ever showing empty/
missing geometry outside the topic's recorded range - is frame_bracket.bracket(),
shared with import_grid_map.py and import_images.py.
"""
import logging

import bpy
import mathutils
import numpy as np

from . import point_cloud_nodes
from .frame_bracket import bracket
from .point_cloud_cache import CachedPointCloudTopic, load_cached_scan
from .tf_tree import TFTree
from .time_alignment import TimeAlignment

logger = logging.getLogger(__name__)

_active_handlers: list = []


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


def _set_float_attr(pc_data: bpy.types.PointCloud, name: str, values: np.ndarray) -> None:
    attr = pc_data.attributes.get(name)
    if attr is None:
        attr = pc_data.attributes.new(name, "FLOAT", "POINT")
    attr.data.foreach_set("value", values.astype(np.float32))


def _apply_scan_pair(pc_data: bpy.types.PointCloud, current: dict[str, np.ndarray],
                      next_scan: dict[str, np.ndarray] | None, rng_seed: int) -> None:
    """Writes BOTH the outgoing ("current") and incoming ("next") scan's points into
    the same Point Cloud at once, tagged with `_pc_src` (0 = outgoing, 1 = incoming)
    and `_pc_rand` (a stable per-point random value in [0, 1)) - point_cloud_nodes.py's
    Geometry Nodes modifier reads these to crossfade between the two. `next_scan` may
    be None (last scan on the topic, or only one scan total), in which case only the
    outgoing scan's points are written and the crossfade modifier is a no-op as long
    as its Blend Factor stays 0 (see create_animated_point_cloud's `update`).

    `rng_seed` should be stable for as long as this particular (current, next) pair
    is being displayed (e.g. the pair's index) so that scrubbing back and forth
    across the same pair always dissolves the same points, rather than reshuffling
    which points vanish/appear on every call.
    """
    cur_xyz = current["xyz"]
    n_cur = cur_xyz.shape[0]
    if next_scan is not None:
        next_xyz = next_scan["xyz"]
        n_next = next_xyz.shape[0]
    else:
        next_xyz = np.zeros((0, 3), dtype=np.float32)
        n_next = 0
    n = n_cur + n_next

    pc_data.resize(n)
    pc_data.attributes["position"].data.foreach_set(
        "vector", np.concatenate([cur_xyz, next_xyz]).astype(np.float32).ravel())

    src = np.concatenate([np.zeros(n_cur, dtype=np.float32), np.ones(n_next, dtype=np.float32)])
    _set_float_attr(pc_data, "_pc_src", src)

    rng = np.random.default_rng(rng_seed)
    _set_float_attr(pc_data, "_pc_rand", rng.random(n).astype(np.float32))

    rgb_cur = current.get("rgb")
    rgb_next = next_scan.get("rgb") if next_scan is not None else None
    if rgb_cur is not None or rgb_next is not None:
        rgba = np.ones((n, 4), dtype=np.float32)
        if rgb_cur is not None:
            rgba[:n_cur, :3] = rgb_cur.astype(np.float32) / 255.0
        if rgb_next is not None:
            rgba[n_cur:, :3] = rgb_next.astype(np.float32) / 255.0
        color_attr = pc_data.attributes.get("Color")
        if color_attr is None:
            color_attr = pc_data.attributes.new("Color", "FLOAT_COLOR", "POINT")
        color_attr.data.foreach_set("color", rgba.ravel())

    field_names = {k[len("field_"):] for k in current if k.startswith("field_")}
    if next_scan is not None:
        field_names |= {k[len("field_"):] for k in next_scan if k.startswith("field_")}
    for name in field_names:
        values = np.zeros(n, dtype=np.float32)
        cur_vals = current.get(f"field_{name}")
        if cur_vals is not None:
            values[:n_cur] = cur_vals
        next_vals = next_scan.get(f"field_{name}") if next_scan is not None else None
        if next_vals is not None:
            values[n_cur:] = next_vals
        _set_float_attr(pc_data, name, values)


def _smoothstep(t: float) -> float:
    """Eases a linear 0-1 fraction into a slow-fast-slow curve so the crossfade
    feels less mechanical than a constant-rate random dissolve."""
    t = min(max(t, 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


def create_animated_point_cloud(topic: str, cache: CachedPointCloudTopic,
                                 empties: dict[str, bpy.types.Object],
                                 collection: bpy.types.Collection,
                                 material: bpy.types.Material) -> bpy.types.Object | None:
    """A single Point Cloud object that always holds the outgoing and incoming
    scan's points together, crossfading between them via point_cloud_nodes.py's
    Geometry Nodes modifier as the timeline moves from one scan's frame to the
    next's - see this module's and point_cloud_nodes.py's docstrings for why (and
    what tradeoff that costs). Reloads the pair's arrays only when the bracketing
    scan changes; the blend factor itself is recomputed and pushed to the modifier
    on every frame so scrubbing within a single pair still animates smoothly.
    """
    if not cache.frame_to_path:
        logger.warning("No cached point cloud scans for %s - skipping", topic)
        return None

    name = topic.strip("/").replace("/", "_")
    pc_data = bpy.data.pointclouds.new(name)
    pc_data.materials.append(material)
    obj = bpy.data.objects.new(name, pc_data)
    collection.objects.link(obj)

    # "intensity" only wins by default when there's no real RGB AND intensity was
    # actually published - preserves "real RGB wins by default" from before this
    # switch was generalized to name an arbitrary field (see point_cloud_nodes.py).
    default_color_field = ("intensity" if not cache.has_real_rgb and "intensity" in cache.field_ranges
                            else "")
    default_field_range = cache.field_ranges.get(default_color_field) if default_color_field else None
    point_cloud_nodes.ensure_crossfade_modifier(
        obj, default_color_field=default_color_field, default_field_range=default_field_range,
    )

    parent = empties.get(cache.frame_id) if cache.frame_id else None
    if parent is not None:
        obj.parent = parent
    else:
        logger.warning("No TF empty found for %s's frame_id '%s'", topic, cache.frame_id)

    frames = sorted(cache.frame_to_path)
    paths = [cache.frame_to_path[f] for f in frames]
    state = {"last_index": -1}

    def update(scene: bpy.types.Scene) -> None:
        idx, raw_factor = bracket(frames, scene.frame_current)
        if idx != state["last_index"]:
            state["last_index"] = idx
            current = load_cached_scan(paths[idx])
            next_scan = load_cached_scan(paths[idx + 1]) if idx + 1 < len(paths) else None
            _apply_scan_pair(pc_data, current, next_scan, rng_seed=idx)

        point_cloud_nodes.set_blend_factor(obj, _smoothstep(raw_factor))

    bpy.app.handlers.frame_change_pre.append(update)
    _active_handlers.append(update)
    update(bpy.context.scene)  # show the right scan(s) for the current frame right away

    return obj


def register() -> None:
    pass


def unregister() -> None:
    for handler in _active_handlers:
        if handler in bpy.app.handlers.frame_change_pre:
            bpy.app.handlers.frame_change_pre.remove(handler)
    _active_handlers.clear()
