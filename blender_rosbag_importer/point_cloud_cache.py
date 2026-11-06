"""Decodes a PointCloud2 topic's messages and caches each scan to disk as a .npz,
keyed by Blender frame number. The animation layer (import_point_clouds.py) loads
these on demand rather than holding every scan's arrays in memory for the whole
import - a long bag can have thousands of scans.
"""
import logging
from dataclasses import dataclass, field as dataclass_field
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from .pointcloud2_decode import decode_pointcloud2
from .time_alignment import TimeAlignment

if TYPE_CHECKING:
    # Only for the type hint below - bag_reader.py imports the `rosbags` package,
    # which this module otherwise has no need for and shouldn't require just to be
    # imported (this file duck-types `bag_reader.messages(...)` rather than actually
    # using the BagReader class), so tests can exercise this module with a plain
    # fake reader double without `rosbags` installed - see tests/test_point_cloud_cache.py.
    from .bag_reader import BagReader

logger = logging.getLogger(__name__)

POINTCLOUD2_MSGTYPE = "sensor_msgs/msg/PointCloud2"


@dataclass
class CachedPointCloudTopic:
    frame_to_path: dict[int, Path]  # Blender frame number -> .npz path, sorted by frame
    frame_id: str | None  # most common header.frame_id seen (for TF parenting)
    has_real_rgb: bool = False  # True if ANY scan on this topic had a real RGB/RGBA
    # field (as opposed to the grayscale-from-intensity fallback, or no color at all).
    # import_point_clouds.py defaults the color-source switch (point_cloud_nodes.py)
    # to "RGB" for such topics and "Intensity" otherwise.
    field_ranges: dict[str, tuple[float, float]] = dataclass_field(default_factory=dict)
    # (min, max) of EVERY extra PointCloud2 field (e.g. "intensity", "ring", "time" -
    # whatever a topic's messages actually carry), across every scan on this topic -
    # generalized from a single `intensity_range` so point_cloud_nodes.py's Color
    # Field switch can seed Field Min/Max for whichever field ends up selected (by
    # default or by hand), not just intensity. Stable for the whole topic rather than
    # renormalized per scan (unlike the old per-scan-normalized decode-time grayscale
    # fallback), so there's no per-scan flicker in how a field's values map to color.


def cache_point_cloud_topic(bag_reader: "BagReader", topic: str, cache_dir: Path,
                             align: TimeAlignment, max_points_per_scan: int | None = None,
                             stride: int = 1) -> CachedPointCloudTopic:
    cache_dir.mkdir(parents=True, exist_ok=True)
    frame_to_path: dict[int, Path] = {}
    frame_id_counts: dict[str, int] = {}
    warned_msgtypes: set[str] = set()
    has_real_rgb = False
    field_ranges: dict[str, tuple[float, float]] = {}

    for i, bag_msg in enumerate(bag_reader.messages(topics=[topic])):
        if bag_msg.msgtype != POINTCLOUD2_MSGTYPE:
            if bag_msg.msgtype not in warned_msgtypes:
                logger.warning("Skipping %s message(s) on '%s' - expected PointCloud2",
                                bag_msg.msgtype, topic)
                warned_msgtypes.add(bag_msg.msgtype)
            continue
        if i % stride != 0:
            continue

        msg = bag_msg.msg
        frame_id_counts[msg.header.frame_id] = frame_id_counts.get(msg.header.frame_id, 0) + 1
        decoded = decode_pointcloud2(msg, max_points=max_points_per_scan)
        has_real_rgb = has_real_rgb or decoded.rgb_is_real
        for name, values in decoded.extra_fields.items():
            if not values.size:
                continue
            lo, hi = float(values.min()), float(values.max())
            if name in field_ranges:
                prev_lo, prev_hi = field_ranges[name]
                lo, hi = min(lo, prev_lo), max(hi, prev_hi)
            field_ranges[name] = (lo, hi)

        bpy_frame = align.frame_of(bag_msg.timestamp_ns)
        path = cache_dir / f"scan_{i:06d}.npz"
        save_kwargs = {"xyz": decoded.xyz}
        if decoded.rgb is not None:
            save_kwargs["rgb"] = decoded.rgb
        for name, values in decoded.extra_fields.items():
            save_kwargs[f"field_{name}"] = values
        np.savez(path, **save_kwargs)
        frame_to_path[bpy_frame] = path

    if len(frame_id_counts) > 1:
        logger.warning("Topic %s published under multiple frame_ids: %s", topic, frame_id_counts)
    frame_id = max(frame_id_counts, key=frame_id_counts.get) if frame_id_counts else None

    return CachedPointCloudTopic(frame_to_path=dict(sorted(frame_to_path.items())), frame_id=frame_id,
                                  has_real_rgb=has_real_rgb, field_ranges=field_ranges)


def load_cached_scan(path: Path) -> dict[str, np.ndarray]:
    with np.load(path) as data:
        return {key: data[key] for key in data.files}
