"""bpy layer: extracts an image topic to a numbered frame sequence on disk and loads
it into Blender as an Image datablock with source='SEQUENCE'.

CompressedImage frames are written verbatim (already JPEG/PNG). Raw Image frames go
through Blender's own image writer (no Pillow/OpenCV dependency needed).
"""
import logging
from pathlib import Path

import bpy
import numpy as np

from .image_decode import compressed_image_bytes, decode_raw_image
from .time_alignment import TimeAlignment

logger = logging.getLogger(__name__)


def _write_raw_frame(rgb: np.ndarray, path: Path) -> None:
    h, w = rgb.shape[:2]
    img = bpy.data.images.new(path.stem, w, h, alpha=False)
    rgba = np.ones((h, w, 4), dtype=np.float32)
    rgba[:, :, :3] = rgb.astype(np.float32) / 255.0
    img.pixels.foreach_set(np.flipud(rgba).ravel())
    img.filepath_raw = str(path)
    img.file_format = "PNG"
    img.save()
    bpy.data.images.remove(img)


def extract_image_sequence(bag_reader, topic: str, output_dir: Path,
                            align: TimeAlignment, stride: int = 1) -> list[Path]:
    """Writes one numbered frame file per message (skipped frames from `align`'s
    time gaps are not backfilled - the sequence is exactly the messages that exist).
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    is_compressed = "Compressed" in bag_reader.topics()[topic].msgtype
    paths: list[Path] = []

    for i, bag_msg in enumerate(bag_reader.messages(topics=[topic])):
        if i % stride != 0:
            continue
        bpy_frame = align.frame_of(bag_msg.timestamp_ns)
        if is_compressed:
            data, ext = compressed_image_bytes(bag_msg.msg)
            path = output_dir / f"frame_{bpy_frame:06d}.{ext}"
            path.write_bytes(data)
        else:
            try:
                rgb = decode_raw_image(bag_msg.msg)
            except ValueError as e:
                logger.warning("Skipping frame on %s: %s", topic, e)
                continue
            path = output_dir / f"frame_{bpy_frame:06d}.png"
            _write_raw_frame(rgb, path)
        paths.append(path)

    return paths


def load_image_sequence(paths: list[Path], name: str) -> bpy.types.Image | None:
    """Loads an extracted frame sequence as a single Blender Image with source='SEQUENCE'."""
    if not paths:
        logger.warning("No frames extracted for image sequence '%s'", name)
        return None
    image = bpy.data.images.load(str(paths[0]))
    image.name = name
    image.source = "SEQUENCE"
    return image


def create_image_empty(image: bpy.types.Image, frame_count: int,
                        collection: bpy.types.Collection, name: str,
                        display_size: float = 0.3) -> bpy.types.Object:
    """An Empty of type 'IMAGE' showing the sequence, parentable to a TF empty like
    point-cloud objects so the image tracks its camera frame's motion.
    """
    empty = bpy.data.objects.new(name, None)
    empty.empty_display_type = "IMAGE"
    empty.empty_display_size = display_size
    empty.data = image
    empty.image_user.frame_duration = frame_count
    empty.image_user.use_auto_refresh = True
    collection.objects.link(empty)
    return empty
