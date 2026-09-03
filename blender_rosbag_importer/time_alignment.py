"""Maps ROS bag timestamps onto Blender frame numbers, anchored to t=0 by default."""
from dataclasses import dataclass


@dataclass
class TimeAlignment:
    start_time_ns: int
    fps: float

    def frame_of(self, timestamp_ns: int) -> int:
        seconds = (timestamp_ns - self.start_time_ns) / 1e9
        return round(seconds * self.fps)

    @classmethod
    def starting_at_zero(cls, start_time_ns: int, fps: float) -> "TimeAlignment":
        return cls(start_time_ns=start_time_ns, fps=fps)
