"""Builds per-child-frame transform time series from a bag's /tf and /tf_static topics.

Each child frame becomes one TFFrame: a sorted list of (timestamp_ns, translation,
quaternion) samples, which the bpy-facing import layer turns into a keyframed Empty.
/tf_static entries are treated as a single constant sample (no interpolation needed -
Blender's own default interpolation between keyframes is enough for /tf's moving frames).
"""
import logging
from dataclasses import dataclass, field

import numpy as np

from .bag_reader import BagReader

logger = logging.getLogger(__name__)

TF_TOPICS = ["/tf", "/tf_static"]


@dataclass
class TFFrame:
    child_frame_id: str
    parent_frame_id: str
    timestamps_ns: list[int] = field(default_factory=list)
    translations: list[tuple[float, float, float]] = field(default_factory=list)
    rotations: list[tuple[float, float, float, float]] = field(default_factory=list)  # xyzw

    def as_arrays(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        order = np.argsort(self.timestamps_ns)
        ts = np.asarray(self.timestamps_ns)[order]
        t = np.asarray(self.translations)[order]
        r = np.asarray(self.rotations)[order]
        return ts, t, r


@dataclass
class TFTree:
    frames: dict[str, TFFrame]

    @classmethod
    def from_bag(cls, reader: BagReader) -> "TFTree":
        available = set(reader.topics()) & set(TF_TOPICS)
        frames: dict[str, TFFrame] = {}
        for bag_msg in reader.messages(topics=sorted(available)):
            is_static = bag_msg.topic == "/tf_static"
            for tfs in bag_msg.msg.transforms:
                child = tfs.child_frame_id
                parent = tfs.header.frame_id
                frame = frames.get(child)
                if frame is None:
                    frame = TFFrame(child_frame_id=child, parent_frame_id=parent)
                    frames[child] = frame
                elif frame.parent_frame_id != parent:
                    logger.warning(
                        "TF frame %s changed parent %s -> %s; keeping first parent",
                        child, frame.parent_frame_id, parent,
                    )

                translation = (tfs.transform.translation.x, tfs.transform.translation.y,
                               tfs.transform.translation.z)
                rotation = (tfs.transform.rotation.x, tfs.transform.rotation.y,
                            tfs.transform.rotation.z, tfs.transform.rotation.w)

                if is_static and frame.timestamps_ns:
                    continue  # static frames only need their first sample
                frame.timestamps_ns.append(bag_msg.timestamp_ns)
                frame.translations.append(translation)
                frame.rotations.append(rotation)

        return cls(frames=frames)

    def root_frames(self) -> set[str]:
        """Frame ids referenced as a parent but never published as a child (e.g. 'map', 'world')."""
        children = set(self.frames)
        parents = {f.parent_frame_id for f in self.frames.values()}
        return parents - children
