"""Hand-rolled sensor_msgs/PointCloud2 -> numpy decoding.

rosbags deserializes PointCloud2 into a message object with a raw `data` byte buffer
plus a `fields` list (name/offset/datatype/count) - it doesn't bundle a
point_cloud2-style helper the way ros_numpy/sensor_msgs_py do, so we build the
structured numpy dtype ourselves from the field layout.
"""
from dataclasses import dataclass, field as dataclass_field

import numpy as np

# sensor_msgs/PointField datatype constants (matches the ROS message definition).
_DATATYPE_TO_NUMPY = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}
_XYZ_FIELDS = {"x", "y", "z"}
_COLOR_FIELDS = {"rgb", "rgba"}


@dataclass
class DecodedPointCloud:
    xyz: np.ndarray  # (N, 3) float32
    rgb: np.ndarray | None  # (N, 3) uint8 - real color field, or a grayscale intensity
    # fallback (see decode_pointcloud2), or None if neither is present.
    rgb_is_real: bool = False  # True if `rgb` came from an actual RGB/RGBA PointField;
    # False if it's the synthesized grayscale-from-intensity fallback (or rgb is None).
    # Consumers (point_cloud_cache.py) use this to default the bpy-side color-source
    # switch (point_cloud_nodes.py) to "RGB" only for topics that really published one.
    extra_fields: dict[str, np.ndarray] = dataclass_field(default_factory=dict)  # (N,) each


def _structured_dtype(msg) -> np.dtype:
    fields = {}
    for f in msg.fields:
        np_type = _DATATYPE_TO_NUMPY[f.datatype]
        fields[f.name] = (np_type, f.offset)
    return np.dtype({"names": list(fields), "formats": [v[0] for v in fields.values()],
                      "offsets": [v[1] for v in fields.values()], "itemsize": msg.point_step})


def _decode_color(raw: np.ndarray, field_names: set, finite: np.ndarray) -> np.ndarray | None:
    if not (_COLOR_FIELDS & field_names):
        return None
    name = "rgb" if "rgb" in field_names else "rgba"
    packed = raw[name][finite]
    # PCL's convention packs RGB into a uint32 but frequently declares the field as
    # FLOAT32 for historical alignment reasons - .view() bit-reinterprets in that case,
    # where a plain .astype() would instead (wrongly) round the float to an int value.
    packed_u32 = packed.view(np.uint32) if packed.dtype == np.float32 else packed.astype(np.uint32)
    r = (packed_u32 >> 16) & 0xFF
    g = (packed_u32 >> 8) & 0xFF
    b = packed_u32 & 0xFF
    return np.stack([r, g, b], axis=-1).astype(np.uint8)


def decode_pointcloud2(msg, max_points: int | None = None) -> DecodedPointCloud:
    """Decodes a sensor_msgs/PointCloud2 message into xyz, a display color, and any
    other scalar fields the message carries (e.g. intensity, ring, time).

    Display color (`rgb`) prefers a real color field; if none exists but an
    `intensity` field does, it's normalized per-scan to a grayscale fallback color -
    real `intensity` values (and any other extra field) are also kept verbatim in
    `extra_fields` regardless of whether they fed the fallback color. `rgb_is_real`
    records which case applied, since bpy-side code (point_cloud_nodes.py) also lets
    the user switch the *displayed* color to a live intensity Color Ramp at any time
    after import - this flag only decides that switch's default, not what's stored.

    `max_points`, if given, uniformly strides the cloud down to roughly that many
    points (decimation for import performance).
    """
    dtype = _structured_dtype(msg)
    raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)

    field_names = {f.name for f in msg.fields}
    xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=-1).astype(np.float32)
    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]

    rgb = _decode_color(raw, field_names, finite)
    rgb_is_real = rgb is not None

    extra_fields = {}
    for name in field_names - _XYZ_FIELDS - _COLOR_FIELDS:
        extra_fields[name] = raw[name][finite].astype(np.float32)

    if rgb is None and "intensity" in extra_fields:
        intensity = extra_fields["intensity"]
        lo, hi = float(intensity.min()), float(intensity.max())
        normalized = (intensity - lo) / (hi - lo) if hi > lo else np.zeros_like(intensity)
        gray = (normalized * 255).astype(np.uint8)
        rgb = np.stack([gray, gray, gray], axis=-1)

    if max_points is not None and xyz.shape[0] > max_points:
        stride = max(1, xyz.shape[0] // max_points)
        xyz = xyz[::stride]
        if rgb is not None:
            rgb = rgb[::stride]
        extra_fields = {name: values[::stride] for name, values in extra_fields.items()}

    return DecodedPointCloud(xyz=xyz, rgb=rgb, rgb_is_real=rgb_is_real, extra_fields=extra_fields)
