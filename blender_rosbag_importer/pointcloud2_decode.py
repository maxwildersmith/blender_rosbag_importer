"""Hand-rolled sensor_msgs/PointCloud2 -> numpy decoding.

rosbags deserializes PointCloud2 into a message object with a raw `data` byte buffer
plus a `fields` list (name/offset/datatype/count) - it doesn't bundle a
point_cloud2-style helper the way ros_numpy/sensor_msgs_py do, so we build the
structured numpy dtype ourselves from the field layout.
"""
from dataclasses import dataclass

import numpy as np

# sensor_msgs/PointField datatype constants (matches the ROS message definition).
_DATATYPE_TO_NUMPY = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}


@dataclass
class DecodedPointCloud:
    xyz: np.ndarray  # (N, 3) float32
    rgb: np.ndarray | None  # (N, 3) uint8, or None if the cloud has no color field


def _structured_dtype(msg) -> np.dtype:
    fields = {}
    for f in msg.fields:
        np_type = _DATATYPE_TO_NUMPY[f.datatype]
        fields[f.name] = (np_type, f.offset)
    return np.dtype({"names": list(fields), "formats": [v[0] for v in fields.values()],
                      "offsets": [v[1] for v in fields.values()], "itemsize": msg.point_step})


def decode_pointcloud2(msg, max_points: int | None = None) -> DecodedPointCloud:
    """Decodes a sensor_msgs/PointCloud2 message into xyz (+ rgb if present).

    `max_points`, if given, uniformly strides the cloud down to roughly that many
    points (decimation for import performance).
    """
    dtype = _structured_dtype(msg)
    raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)

    field_names = {f.name for f in msg.fields}
    xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=-1).astype(np.float32)

    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]

    rgb = None
    if "rgb" in field_names or "rgba" in field_names:
        packed = raw["rgb"] if "rgb" in field_names else raw["rgba"]
        packed_u32 = packed[finite].astype(np.uint32).view(np.uint32)
        r = (packed_u32 >> 16) & 0xFF
        g = (packed_u32 >> 8) & 0xFF
        b = packed_u32 & 0xFF
        rgb = np.stack([r, g, b], axis=-1).astype(np.uint8)

    if max_points is not None and xyz.shape[0] > max_points:
        stride = max(1, xyz.shape[0] // max_points)
        xyz = xyz[::stride]
        if rgb is not None:
            rgb = rgb[::stride]

    return DecodedPointCloud(xyz=xyz, rgb=rgb)
