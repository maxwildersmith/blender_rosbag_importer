"""Pure-python (no bpy import) helper shared by every per-frame playback handler
(import_point_clouds.py, import_grid_map.py, import_images.py): given a sorted list
of Blender frame numbers - one per cached scan/mesh/image, keyed the same way
CachedPointCloudTopic.frame_to_path/CachedGridMapTopic.frame_to_path/an image
sequence's per-frame list all are - and the current scene frame, finds which entry
to hold and (only point clouds' crossfade actually uses this) how far between it and
the next entry the current frame sits.

This consolidates what used to be three near-identical `bisect_right(...) - 1`
snippets (one per importer) into a single implementation, so the hold-the-first-
entry-before-the-first-frame / hold-the-last-entry-forever-after-the-last-frame
behavior - the thing that stops any modality from ever showing empty/missing data
outside its own recorded frame range - is defined and tested in exactly one place,
rather than three copies that could silently drift apart from each other over time.
"""
import bisect


def bracket(frames: list, scene_frame: float) -> tuple:
    """Returns `(idx, factor)` for a `scene_frame` against a non-empty, ascending
    `frames` list:

    - `idx` is the index into `frames` of the most recent entry at-or-before
      `scene_frame`, clamped to 0 - so a `scene_frame` before `frames[0]` holds
      index 0 (the first entry) rather than going negative, and a `scene_frame` at
      or after `frames[-1]` holds `len(frames) - 1` (the last entry) rather than
      ever running off the end of the list. Every caller only needs `idx` to look
      up "the scan/frame to show right now."
    - `factor` is how far, as a 0-1 fraction, `scene_frame` sits between
      `frames[idx]` and `frames[idx + 1]` - `0.0` if there is no next entry (`idx`
      is the last one) or the two entries share the same frame number (a
      zero-length span). Only import_point_clouds.py's crossfade actually uses this;
      import_grid_map.py and import_images.py just ignore it (`_`).

    `frames` must be non-empty and sorted ascending (every caller already keeps its
    frame list this way, e.g. `sorted(cache.frame_to_path)`).
    """
    idx = max(bisect.bisect_right(frames, scene_frame) - 1, 0)
    if idx + 1 < len(frames) and frames[idx + 1] > frames[idx]:
        factor = (scene_frame - frames[idx]) / (frames[idx + 1] - frames[idx])
        factor = min(max(factor, 0.0), 1.0)
    else:
        factor = 0.0
    return idx, factor


def image_user_frame_offset(idx: int, scene_frame: float, frame_start: int = 1) -> float:
    """Returns the `bpy.types.ImageUser.frame_offset` value that makes an ImageUser
    with `image_user.frame_start == frame_start` display its `idx`-th (0-based)
    sequence file - the file `bracket()` says to hold - at `scene_frame`.

    Blender computes the displayed (1-based) sequence file index as
    `scene_frame - frame_start + 1 + frame_offset` (internally clamped to
    `[1, frame_duration]` when the sequence isn't set to loop) - this is just that
    formula solved for `frame_offset` given the desired 1-based index `idx + 1`.
    Used by import_images.py to hold each message's frame's image exactly like
    point clouds/grid maps hold their scans, instead of trusting Blender's own
    sequence player to advance one file per Blender frame (see that module's
    docstring for why that default is wrong here).
    """
    return (idx + 1) - scene_frame + frame_start - 1
