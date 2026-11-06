"""bpy layer: extracts an image topic to a numbered frame sequence on disk and loads
it into Blender as an Image datablock with source='SEQUENCE', animated the same way
point clouds and grid maps are - held at whichever message's frame most recently
passed, via a frame_change_pre handler using frame_bracket.bracket() - rather than
trusting Blender's own image-sequence player to just advance one file per Blender
frame, which is what this used to do and is wrong on two counts:

1. **Wrong file naming caused a real pink/magenta "missing image" flash.** Files
   used to be named by their (generally sparse, irregularly-spaced) Blender frame
   number, e.g. `frame_000000.png`, `frame_000042.png`, `frame_000100.png`. Blender's
   image-sequence file discovery does NOT scan the directory for whatever files
   happen to exist and play them in order - it takes the numeric suffix of the
   FIRST loaded file and looks for the NEXT sequence entry by incrementing that
   number by exactly 1 per Blender frame advanced. So after loading
   `frame_000000.png`, Blender goes looking for `frame_000001.png`,
   `frame_000002.png`, ... - and since messages don't arrive on every single
   Blender frame, almost none of those consecutively-numbered files ever existed on
   disk, each producing a genuine missing-file load failure - Blender's classic pink
   texture. Frames are now named by their plain 1-based position in the sequence
   (`frame_000001.png`, `frame_000002.png`, ...), which always exist consecutively,
   so Blender's own file discovery always finds a real file.
2. **Wrong (and desynced) timing.** Advancing exactly one sequence file per Blender
   frame has nothing to do with a topic's actual message timing (or the shared
   TimeAlignment every other modality uses) - so images silently drifted out of sync
   with point clouds/grid maps rather than holding the same real-world instant a
   viewer at "1 second" would expect from them. The frame_change_pre handler below
   fixes this by driving `image_user.frame_offset` every frame from the SAME
   bisect-based hold-last-value logic (frame_bracket.bracket()) point clouds and
   grid maps use, computed from each frame's real `align.frame_of(...)` Blender
   frame number (extract_image_sequence still records that per file, just no longer
   encodes it into the filename).

CompressedImage frames are written verbatim (already JPEG/PNG). Raw Image frames go
through Blender's own image writer (no Pillow/OpenCV dependency needed).
"""
import logging
from dataclasses import dataclass, field as dataclass_field
from pathlib import Path

import bpy
import numpy as np

from .frame_bracket import bracket, image_user_frame_offset
from .image_decode import compressed_image_bytes, decode_raw_image
from .time_alignment import TimeAlignment

logger = logging.getLogger(__name__)

_active_handlers: list = []


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


@dataclass
class ExtractedImageSequence:
    paths: list = dataclass_field(default_factory=list)  # on-disk order == message order
    frames: list = dataclass_field(default_factory=list)  # parallel: each path's Blender frame


def extract_image_sequence(bag_reader, topic: str, output_dir: Path,
                            align: TimeAlignment, stride: int = 1) -> ExtractedImageSequence:
    """Writes one numbered frame file per message, in message order, and returns
    both the file paths and the Blender frame each one corresponds to (skipped
    frames from `align`'s time gaps are not backfilled - the sequence is exactly the
    messages that exist).

    Files are named by their 1-based POSITION in the sequence, NOT by their Blender
    frame number - see this module's docstring for why naming them by Blender frame
    directly caused a pink/magenta "missing image" flash.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    expected_msgtype = bag_reader.topics()[topic].msgtype
    is_compressed = "Compressed" in expected_msgtype
    result = ExtractedImageSequence()
    warned_msgtypes: set = set()

    for i, bag_msg in enumerate(bag_reader.messages(topics=[topic])):
        if bag_msg.msgtype != expected_msgtype:
            if bag_msg.msgtype not in warned_msgtypes:
                logger.warning("Skipping %s message(s) on '%s' - expected %s",
                                bag_msg.msgtype, topic, expected_msgtype)
                warned_msgtypes.add(bag_msg.msgtype)
            continue
        if i % stride != 0:
            continue
        bpy_frame = align.frame_of(bag_msg.timestamp_ns)
        seq_index = len(result.paths) + 1  # 1-based, consecutive - what Blender's own
        # image-sequence file discovery expects to find on disk (see module docstring).
        if is_compressed:
            data, ext = compressed_image_bytes(bag_msg.msg)
            path = output_dir / f"frame_{seq_index:06d}.{ext}"
            path.write_bytes(data)
        else:
            try:
                rgb = decode_raw_image(bag_msg.msg)
            except ValueError as e:
                logger.warning("Skipping frame on %s: %s", topic, e)
                continue
            path = output_dir / f"frame_{seq_index:06d}.png"
            _write_raw_frame(rgb, path)
        result.paths.append(path)
        result.frames.append(bpy_frame)

    return result


def load_image_sequence(sequence: ExtractedImageSequence, name: str) -> bpy.types.Image | None:
    """Loads an extracted frame sequence as a single Blender Image with source='SEQUENCE'."""
    if not sequence.paths:
        logger.warning("No frames extracted for image sequence '%s'", name)
        return None
    image = bpy.data.images.load(str(sequence.paths[0]))
    image.name = name
    image.source = "SEQUENCE"
    return image


def create_image_empty(image: bpy.types.Image, sequence: ExtractedImageSequence,
                        collection: bpy.types.Collection, name: str,
                        display_size: float = 0.3) -> bpy.types.Object:
    """An Empty of type 'IMAGE' showing the sequence, parentable to a TF empty like
    point-cloud objects so the image tracks its camera frame's motion. A
    frame_change_pre handler drives `image_user.frame_offset` every frame so the
    displayed file always matches frame_bracket.bracket()'s hold-last-value choice
    for the current scene frame against `sequence.frames` - see this module's
    docstring for why Blender's own default sequence advancement (one file per
    Blender frame) is wrong here.
    """
    empty = bpy.data.objects.new(name, None)
    empty.empty_display_type = "IMAGE"
    empty.empty_display_size = display_size
    empty.data = image
    empty.image_user.frame_start = 1
    empty.image_user.frame_duration = len(sequence.paths)
    empty.image_user.use_auto_refresh = True
    collection.objects.link(empty)

    frames = sequence.frames

    def update(scene: bpy.types.Scene) -> None:
        idx, _ = bracket(frames, scene.frame_current)
        # Recomputed unconditionally every frame (not just when idx changes) since
        # image_user_frame_offset depends on scene_frame directly - unlike point
        # clouds/grid maps there's no expensive array reload to skip here, just one
        # int property write.
        empty.image_user.frame_offset = image_user_frame_offset(idx, scene.frame_current)

    bpy.app.handlers.frame_change_pre.append(update)
    _active_handlers.append(update)
    update(bpy.context.scene)  # show the right frame for the current frame right away

    return empty


def register() -> None:
    pass


def unregister() -> None:
    for handler in _active_handlers:
        if handler in bpy.app.handlers.frame_change_pre:
            bpy.app.handlers.frame_change_pre.remove(handler)
    _active_handlers.clear()
