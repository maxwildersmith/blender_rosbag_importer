"""Decodes a GridMap topic's messages and caches each scan to disk as a .npz, keyed
by Blender frame number - mirrors point_cloud_cache.py's approach exactly (see that
module's docstring for the "why cache to disk instead of holding everything in
memory" rationale, which applies here just as much).
"""
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from .grid_map_decode import decode_grid_map
from .time_alignment import TimeAlignment

if TYPE_CHECKING:
    # Only for the type hint below - see point_cloud_cache.py's identical guard for
    # why this doesn't just import bag_reader.BagReader unconditionally.
    from .bag_reader import BagReader

logger = logging.getLogger(__name__)

GRIDMAP_MSGTYPE = "grid_map_msgs/msg/GridMap"


@dataclass
class CachedGridMapTopic:
    frame_to_path: dict[int, Path]  # Blender frame number -> .npz path, sorted by frame
    frame_id: str | None  # most common header.frame_id seen (for TF parenting)
    rows: int = 0  # grid size at the FIRST scan - import_grid_map.py builds the mesh
    cols: int = 0  # topology once from this and warns/skips any later scan whose
    # rows/cols don't match, rather than trying to rebuild mesh topology every frame.
    resolution: float = 0.0  # meters/cell, from the FIRST scan - like rows/cols,
    length_x: float = 0.0  # assumed constant for the topic so the mesh's per-vertex
    length_y: float = 0.0  # (x, y) layout can be computed once rather than every frame.
    has_real_color: bool = False  # True if ANY scan had a real packed "color" layer -
    # import_grid_map.py defaults grid_map_nodes.py's color-source switch to "packed
    # color" for such topics and "Elevation ramp" otherwise (mirrors
    # CachedPointCloudTopic.has_real_rgb).
    elevation_range: tuple[float, float] | None = None  # (min, max) of the elevation
    # layer across every scan on this topic (over VALID cells only) - seeds the
    # color-ramp modifier's Elevation Min/Max inputs (mirrors
    # CachedPointCloudTopic.field_ranges, generalized there to arbitrary fields since
    # a point cloud can carry several candidate color-driving fields; a grid map only
    # ever has the one, "elevation", so this stays a single named range).


def cache_grid_map_topic(bag_reader: "BagReader", topic: str, cache_dir: Path,
                          align: TimeAlignment, stride: int = 1) -> CachedGridMapTopic:
    cache_dir.mkdir(parents=True, exist_ok=True)
    frame_to_path: dict[int, Path] = {}
    frame_id_counts: dict[str, int] = {}
    warned_msgtypes: set[str] = set()
    rows = cols = 0
    resolution = length_x = length_y = 0.0
    has_real_color = False
    elevation_lo: float | None = None
    elevation_hi: float | None = None

    for i, bag_msg in enumerate(bag_reader.messages(topics=[topic])):
        if bag_msg.msgtype != GRIDMAP_MSGTYPE:
            if bag_msg.msgtype not in warned_msgtypes:
                logger.warning("Skipping %s message(s) on '%s' - expected GridMap",
                                bag_msg.msgtype, topic)
                warned_msgtypes.add(bag_msg.msgtype)
            continue
        if i % stride != 0:
            continue

        msg = bag_msg.msg
        frame_id_counts[msg.header.frame_id] = frame_id_counts.get(msg.header.frame_id, 0) + 1
        decoded = decode_grid_map(msg)
        if rows == 0:
            rows, cols = decoded.rows, decoded.cols
            resolution, length_x, length_y = decoded.resolution, decoded.length_x, decoded.length_y
        elif (decoded.rows, decoded.cols) != (rows, cols):
            logger.warning("Topic %s's grid size changed (%dx%d -> %dx%d); "
                            "import_grid_map.py will skip scans that don't match "
                            "the first one's size", topic, rows, cols, decoded.rows, decoded.cols)

        has_real_color = has_real_color or decoded.rgb_is_real
        valid_elevation = decoded.elevation[decoded.valid]
        if valid_elevation.size:
            lo, hi = float(valid_elevation.min()), float(valid_elevation.max())
            elevation_lo = lo if elevation_lo is None else min(elevation_lo, lo)
            elevation_hi = hi if elevation_hi is None else max(elevation_hi, hi)

        bpy_frame = align.frame_of(bag_msg.timestamp_ns)
        path = cache_dir / f"scan_{i:06d}.npz"
        save_kwargs = {
            "rows": decoded.rows, "cols": decoded.cols,
            "elevation": decoded.elevation, "valid": decoded.valid,
            "position": np.array(decoded.position, dtype=np.float32),
            "orientation": np.array(decoded.orientation, dtype=np.float32),
        }
        if decoded.rgb is not None:
            save_kwargs["rgb"] = decoded.rgb
        for name, values in decoded.extra_layers.items():
            save_kwargs[f"field_{name}"] = values
        np.savez(path, **save_kwargs)
        frame_to_path[bpy_frame] = path

    if len(frame_id_counts) > 1:
        logger.warning("Topic %s published under multiple frame_ids: %s", topic, frame_id_counts)
    frame_id = max(frame_id_counts, key=frame_id_counts.get) if frame_id_counts else None
    elevation_range = (elevation_lo, elevation_hi) if elevation_lo is not None else None

    return CachedGridMapTopic(frame_to_path=dict(sorted(frame_to_path.items())), frame_id=frame_id,
                               rows=rows, cols=cols, resolution=resolution,
                               length_x=length_x, length_y=length_y,
                               has_real_color=has_real_color, elevation_range=elevation_range)


def load_cached_grid_scan(path: Path) -> dict[str, np.ndarray]:
    with np.load(path) as data:
        return {key: data[key] for key in data.files}
