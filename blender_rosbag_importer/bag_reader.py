"""Thin wrapper around rosbags.highlevel.AnyReader: list topics, iterate messages
in timestamp order. Kept free of bpy so it can be unit-tested outside Blender.
"""
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, NamedTuple

from rosbags.highlevel import AnyReader


@dataclass
class TopicInfo:
    name: str
    msgtype: str
    count: int


class BagMessage(NamedTuple):
    topic: str
    msgtype: str
    timestamp_ns: int
    msg: object


class BagReader:
    """Reads a single ROS1 .bag or ROS2 .db3/mcap bag via rosbags."""

    def __init__(self, bag_path: Path) -> None:
        self.bag_path = Path(bag_path)
        if not self.bag_path.exists():
            raise FileNotFoundError(f"{self.bag_path} does not exist.")

    def topics(self) -> dict[str, TopicInfo]:
        with AnyReader([self.bag_path]) as reader:
            return {
                c.topic: TopicInfo(name=c.topic, msgtype=c.msgtype, count=c.msgcount)
                for c in reader.connections
            }

    def messages(self, topics: list[str]) -> Iterator[BagMessage]:
        """Yields messages for the given topics, in timestamp order (AnyReader's default)."""
        with AnyReader([self.bag_path]) as reader:
            connections = [c for c in reader.connections if c.topic in topics]
            for connection, timestamp_ns, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                yield BagMessage(connection.topic, connection.msgtype, timestamp_ns, msg)

    def start_time_ns(self, topics: list[str]) -> int:
        """Earliest timestamp among the given topics. Raises if none of them have messages."""
        earliest = None
        with AnyReader([self.bag_path]) as reader:
            connections = [c for c in reader.connections if c.topic in topics]
            for connection, timestamp_ns, _ in reader.messages(connections=connections):
                earliest = timestamp_ns
                break
        if earliest is None:
            raise ValueError(f"No messages found on topics {topics}")
        return earliest

    def end_time_ns(self, topics: list[str]) -> int:
        """Latest timestamp among the given topics. Raises if none of them have messages."""
        latest = None
        for bag_msg in self.messages(topics=topics):
            latest = bag_msg.timestamp_ns
        if latest is None:
            raise ValueError(f"No messages found on topics {topics}")
        return latest
