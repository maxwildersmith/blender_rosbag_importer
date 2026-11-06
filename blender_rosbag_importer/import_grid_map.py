"""bpy layer: creates one animated Mesh object per GridMap topic - the full rows x
cols grid of vertices/quads is built ONCE at import time (grid geometry - i.e.
resolution/length_x/length_y - is assumed constant for a topic, exactly like
PointCloud2's header.frame_id assumption elsewhere in this add-on), and a
frame_change_pre handler then just rewrites per-vertex Z/color/validity/layer
attributes via foreach_set every frame the bracketing scan changes (mirroring
import_point_clouds.py's approach almost exactly). grid_map_nodes.py's shared
Geometry Nodes modifier turns invalid cells into real holes and switches the
displayed color between a packed color layer and a live elevation Color Ramp.

Unlike point clouds, GridMap scans on one topic share the same vertex grid from
message to message, so there's no crossfade here - see grid_map_nodes.py's
docstring for why a hard swap is used instead (and why a lerp would be easy to add
later if ever wanted). Which scan brackets the current frame - holding the first
scan before its frame and the last one forever after its frame - is
frame_bracket.bracket(), shared with import_point_clouds.py and import_images.py.

The map's own local pose (`info.pose`, an offset the grid's center may have from
its `header.frame_id`'s own origin - e.g. a robot-centered local elevation map that
recenters itself as it walks) is applied as this object's `location`/
`rotation_quaternion` every frame too, on top of parenting to the TF empty matching
`header.frame_id` - mirroring create_tf_empties' ROS (x, y, z, w) -> Blender
(w, x, y, z) quaternion reordering.
"""
import logging

import bpy
import mathutils
import numpy as np

from . import grid_map_nodes
from .frame_bracket import bracket
from .grid_map_cache import CachedGridMapTopic, load_cached_grid_scan
from .grid_map_decode import cell_center_local_xy

logger = logging.getLogger(__name__)

_active_handlers: list = []


def _build_grid_mesh(rows: int, cols: int) -> tuple[bpy.types.Mesh, list]:
    """Builds the full rows x cols grid's fixed topology once: one vertex per cell
    (initial position is a placeholder - overwritten every frame) and one quad per
    2x2 neighborhood of cells. Returns the mesh and the vertex-index-by-(row, col)
    layout implied by `idx(i, j) = i * cols + j`, used by the per-frame writer.
    """
    n = rows * cols
    vertices = [(0.0, 0.0, 0.0)] * n
    faces = []
    for i in range(rows - 1):
        for j in range(cols - 1):
            a = i * cols + j
            b = (i + 1) * cols + j
            c = (i + 1) * cols + (j + 1)
            d = i * cols + (j + 1)
            faces.append((a, b, c, d))

    mesh = bpy.data.meshes.new("grid_map")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    return mesh, faces


def _set_point_float_attr(mesh: bpy.types.Mesh, name: str, values: np.ndarray) -> None:
    attr = mesh.attributes.get(name)
    if attr is None:
        attr = mesh.attributes.new(name, "FLOAT", "POINT")
    attr.data.foreach_set("value", values.astype(np.float32))


def _apply_grid_scan(mesh: bpy.types.Mesh, xy: tuple, arrays: dict) -> bool:
    """Overwrites `mesh`'s per-vertex position/validity/color/layer attributes from
    one cached scan. Returns False (and changes nothing) if this scan's grid size
    doesn't match the mesh's fixed topology - see grid_map_cache.py's warning about
    a topic whose grid size changes mid-recording.
    """
    xx, yy = xy
    rows, cols = int(arrays["rows"]), int(arrays["cols"])
    n = rows * cols
    if len(mesh.vertices) != n:
        return False

    elevation = arrays["elevation"]
    z = np.nan_to_num(elevation, nan=0.0).astype(np.float32)
    positions = np.stack([xx, yy, z], axis=-1).reshape(n, 3)
    mesh.vertices.foreach_set("co", positions.ravel())

    valid = arrays["valid"].reshape(n).astype(np.float32)
    _set_point_float_attr(mesh, "_valid", valid)
    _set_point_float_attr(mesh, "elevation", z.reshape(n))

    if "rgb" in arrays:
        rgba = np.ones((n, 4), dtype=np.float32)
        rgba[:, :3] = arrays["rgb"].reshape(n, 3).astype(np.float32) / 255.0
        color_attr = mesh.attributes.get("Color")
        if color_attr is None:
            color_attr = mesh.attributes.new("Color", "FLOAT_COLOR", "POINT")
        color_attr.data.foreach_set("color", rgba.ravel())

    for key, values in arrays.items():
        if not key.startswith("field_"):
            continue
        name = key[len("field_"):]
        flat = np.nan_to_num(values, nan=0.0).reshape(n).astype(np.float32)
        _set_point_float_attr(mesh, name, flat)

    mesh.update()
    return True


def create_animated_grid_map(topic: str, cache: CachedGridMapTopic,
                              empties: dict[str, bpy.types.Object],
                              collection: bpy.types.Collection,
                              material: bpy.types.Material) -> bpy.types.Object | None:
    """A single Mesh object whose vertex data is rewritten in place as the timeline
    scrubs - see this module's docstring for the overall approach.
    """
    if not cache.frame_to_path or cache.rows < 2 or cache.cols < 2:
        logger.warning("No usable cached grid map scans for %s - skipping", topic)
        return None

    name = topic.strip("/").replace("/", "_")
    mesh, _faces = _build_grid_mesh(cache.rows, cache.cols)
    mesh.materials.append(material)
    obj = bpy.data.objects.new(name, mesh)
    obj.rotation_mode = "QUATERNION"
    collection.objects.link(obj)

    grid_map_nodes.ensure_grid_map_modifier(
        obj, default_use_elevation=not cache.has_real_color, elevation_range=cache.elevation_range,
    )

    parent = empties.get(cache.frame_id) if cache.frame_id else None
    if parent is not None:
        obj.parent = parent
    else:
        logger.warning("No TF empty found for %s's frame_id '%s'", topic, cache.frame_id)

    xy = cell_center_local_xy(cache.rows, cache.cols, cache.resolution, cache.length_x, cache.length_y)

    frames = sorted(cache.frame_to_path)
    paths = [cache.frame_to_path[f] for f in frames]
    state = {"last_index": -1, "warned_mismatch": False}

    def update(scene: bpy.types.Scene) -> None:
        idx, _ = bracket(frames, scene.frame_current)
        if idx == state["last_index"]:
            return
        state["last_index"] = idx
        arrays = load_cached_grid_scan(paths[idx])
        ok = _apply_grid_scan(mesh, xy, arrays)
        if not ok:
            if not state["warned_mismatch"]:
                logger.warning("%s: scan at frame %d has a different grid size than "
                                "the topic's first scan - skipping until it matches again",
                                topic, frames[idx])
                state["warned_mismatch"] = True
            return
        obj.location = mathutils.Vector(tuple(arrays["position"]))
        x, y, z, w = arrays["orientation"]
        obj.rotation_quaternion = mathutils.Quaternion((w, x, y, z))

    bpy.app.handlers.frame_change_pre.append(update)
    _active_handlers.append(update)
    update(bpy.context.scene)  # show the right scan for the current frame right away

    return obj


def register() -> None:
    pass


def unregister() -> None:
    for handler in _active_handlers:
        if handler in bpy.app.handlers.frame_change_pre:
            bpy.app.handlers.frame_change_pre.remove(handler)
    _active_handlers.clear()
