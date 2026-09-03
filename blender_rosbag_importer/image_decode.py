"""Pure-python sensor_msgs/Image and sensor_msgs/CompressedImage -> numpy/bytes decoding.

CompressedImage already contains ready-to-write JPEG/PNG bytes, so that path needs no
decoding at all. Raw Image messages carry a handful of common ROS encodings; anything
else is rejected explicitly (a bag's encodings are an external boundary - unrecognized
values should fail loudly rather than silently produce garbage pixels).
"""
import numpy as np

_RAW_ENCODINGS_TO_CHANNELS = {"rgb8": 3, "bgr8": 3, "mono8": 1, "mono16": 1}


def decode_raw_image(msg) -> np.ndarray:
    """Returns an (H, W, 3) uint8 RGB array for a sensor_msgs/Image."""
    encoding = msg.encoding
    if encoding not in _RAW_ENCODINGS_TO_CHANNELS:
        raise ValueError(f"Unsupported Image encoding '{encoding}'")

    channels = _RAW_ENCODINGS_TO_CHANNELS[encoding]
    dtype = np.uint16 if encoding == "mono16" else np.uint8
    arr = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(msg.height, msg.width, channels)

    if encoding == "bgr8":
        arr = arr[:, :, ::-1]
    if encoding == "mono16":
        arr = (arr.astype(np.float32) / arr.max() * 255).astype(np.uint8) if arr.max() > 0 else arr.astype(np.uint8)
    if channels == 1:
        arr = np.repeat(arr[:, :, :1].astype(np.uint8), 3, axis=2)

    return np.ascontiguousarray(arr)


def compressed_image_bytes(msg) -> tuple[bytes, str]:
    """Returns (raw file bytes, file extension) for a sensor_msgs/CompressedImage."""
    fmt = msg.format.lower()
    ext = "png" if "png" in fmt else "jpg"
    return bytes(msg.data), ext
