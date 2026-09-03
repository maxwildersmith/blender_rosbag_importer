"""Pure-python tests, runnable outside Blender (no bpy import anywhere in this file)."""
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from blender_rosbag_importer.pointcloud2_decode import decode_pointcloud2


def _field(name, offset, datatype, count=1):
    return SimpleNamespace(name=name, offset=offset, datatype=datatype, count=count)


def make_xyzrgb_msg(points_xyz, colors_rgb):
    fields = [
        _field("x", 0, 7), _field("y", 4, 7), _field("z", 8, 7),
        _field("rgb", 16, 6),
    ]
    point_step = 20
    data = bytearray()
    for (x, y, z), (r, g, b) in zip(points_xyz, colors_rgb):
        packed = (r << 16) | (g << 8) | b
        data += struct.pack("<fff", x, y, z)
        data += b"\x00" * 4  # padding to offset 16
        data += struct.pack("<I", packed)
    return SimpleNamespace(fields=fields, point_step=point_step, width=len(points_xyz),
                            height=1, data=bytes(data))


def test_decode_xyz_and_rgb():
    pts = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (float("nan"), 0.0, 0.0)]
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    msg = make_xyzrgb_msg(pts, colors)
    decoded = decode_pointcloud2(msg)
    assert decoded.xyz.shape == (2, 3), decoded.xyz.shape
    assert np.allclose(decoded.xyz[0], [1.0, 2.0, 3.0])
    assert np.allclose(decoded.xyz[1], [4.0, 5.0, 6.0])
    assert decoded.rgb.shape == (2, 3)
    assert tuple(decoded.rgb[0]) == (255, 0, 0)
    assert tuple(decoded.rgb[1]) == (0, 255, 0)


def test_decode_decimation():
    pts = [(float(i), 0.0, 0.0) for i in range(100)]
    colors = [(0, 0, 0)] * 100
    msg = make_xyzrgb_msg(pts, colors)
    decoded = decode_pointcloud2(msg, max_points=10)
    assert decoded.xyz.shape[0] <= 10


if __name__ == "__main__":
    test_decode_xyz_and_rgb()
    test_decode_decimation()
    print("OK")
