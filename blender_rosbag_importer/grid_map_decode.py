"""Hand-rolled grid_map_msgs/GridMap -> numpy decoding (no bpy import - see
pointcloud2_decode.py for the same split rationale). `grid_map_msgs` (ETH/ANYbotics'
`grid_map` ROS package - the standard elevation-map message in legged-robotics work)
isn't a core ROS package, so `rosbags`' typestore doesn't bundle it the way it does
sensor_msgs/geometry_msgs - but bag_reader.py doesn't need it to either: for a ROS1
`.bag`, `rosbags.highlevel.AnyReader` parses any connection's message definition
straight out of the bag file itself (ROS1 bags embed the full .msg text per
connection), and this module - like pointcloud2_decode.py - never imports the
generated message class anyway, just duck-types whatever attributes the deserialized
object exposes. So as long as a bag really contains grid_map_msgs/GridMap messages,
this decodes them the same way regardless of how `rosbags` produced the object.

**Unverified assumption - the circular buffer.** grid_map internally stores each
layer as a fixed-size buffer that's logically "rolled" in place (so shifting the map
- e.g. as a robot walks and a local elevation map re-centers - doesn't require
copying the whole grid): `outer_start_index`/`inner_start_index` on the message say
which buffer cell holds logical cell (0, 0). This module assumes, from recollection
of `grid_map_ros`'s `GridMapRosConverter`/`GridMapMsgHelpers` conversion code (no
network access to re-check the source, and no grid_map_msgs bag was found anywhere
on this machine to inspect directly - see README's verification section):

- Each layer's `std_msgs/Float32MultiArray.data` is row-major with
  `layout.dim[0].size` rows (grid_map's "outer"/x-direction) and `layout.dim[1].size`
  columns ("inner"/y-direction): `raw[row, col] = data[row * dim[1].size + col]`.
- Logical cell `(i, j)` - `i` a row 0..rows-1 with 0 = the *maximum-x* edge of the
  map, `j` a column 0..cols-1 with 0 = the *maximum-y* edge - lives at buffer
  position `((i + outer_start_index) % rows, (j + inner_start_index) % cols)`.

If this is wrong, the visible symptom is exactly what the task description warns
about: a heightfield that looks shifted/torn along one or both axes wherever a
map's roll offset isn't (0, 0) - which for many recording setups (a map that starts
fresh each recording, or one that's re-seeded often) may simply never show up, since
the buffer is never rolled far enough to matter. `_unwrap_layer()` isolates this
exact assumption in one small function so it's a one-line fix (swap the sign of the
modulo offset, or swap which axis is "outer" vs "inner") if a real grid_map bag ever
proves it wrong.
"""
from dataclasses import dataclass, field as dataclass_field

import numpy as np


@dataclass
class DecodedGridMap:
    rows: int  # buffer/grid size along grid_map's "outer" (x) direction
    cols: int  # ... along its "inner" (y) direction
    resolution: float  # meters/cell
    length_x: float  # total map extent, meters
    length_y: float
    position: tuple[float, float, float]  # info.pose.position - map center's offset
    # within header.frame_id (NOT necessarily the frame's own origin)
    orientation: tuple[float, float, float, float]  # info.pose.orientation, ROS xyzw
    elevation: np.ndarray  # (rows, cols) float32, unwrapped - NaN where invalid/holes
    valid: np.ndarray  # (rows, cols) bool - elevation finite AND every basic layer
    # finite at that cell (or just elevation's own finiteness if there are no basic
    # layers) - cells where this is False should render as a gap, not z=0.
    rgb: np.ndarray | None  # (rows, cols, 3) uint8 from a packed "color" layer if one
    # exists (grid_map's convention for RGB is the same float32-reinterpreted-as-
    # packed-uint32 trick PCL/PointCloud2 use - see pointcloud2_decode._decode_color),
    # else None.
    rgb_is_real: bool  # True if `rgb` came from a real "color" layer - mirrors
    # pointcloud2_decode.DecodedPointCloud.rgb_is_real; grid maps have no
    # intensity-style grayscale fallback baked in here (the elevation Color Ramp
    # plays that role live, in grid_map_nodes.py), so this is really just "is there
    # a color layer at all".
    extra_layers: dict[str, np.ndarray] = dataclass_field(default_factory=dict)  # every
    # other layer (e.g. "traversability", "variance"), each (rows, cols) float32,
    # unwrapped the same way as elevation.


def _unwrap_layer(flat, rows: int, cols: int, outer_start: int, inner_start: int) -> np.ndarray:
    """Rebuilds a (rows, cols) row-major array from grid_map's rolling-buffer wire
    format. See this module's docstring for the (unverified) convention assumed.
    """
    buffer = np.asarray(flat, dtype=np.float32).reshape(rows, cols)
    row_idx = (np.arange(rows) + int(outer_start)) % rows
    col_idx = (np.arange(cols) + int(inner_start)) % cols
    return buffer[row_idx][:, col_idx]


def _pack_color_layer(values: np.ndarray) -> np.ndarray:
    """Unpacks a grid_map "color" layer (float32-reinterpreted-as-packed-RGB, same
    bit layout PCL/PointCloud2 use) into (rows, cols, 3) uint8. NaN/non-finite cells
    (no color assigned to that cell) come back as black - see decode_grid_map's note
    on why that doesn't by itself punch a hole.
    """
    finite = np.isfinite(values)
    packed_u32 = np.zeros(values.shape, dtype=np.uint32)
    packed_u32[finite] = values[finite].astype(np.float32).view(np.uint32)
    r = (packed_u32 >> 16) & 0xFF
    g = (packed_u32 >> 8) & 0xFF
    b = packed_u32 & 0xFF
    return np.stack([r, g, b], axis=-1).astype(np.uint8)


def cell_center_local_xy(rows: int, cols: int, resolution: float,
                          length_x: float, length_y: float) -> tuple[np.ndarray, np.ndarray]:
    """Returns (rows, cols) arrays of each cell's CENTER position (x, y), in the
    frame defined by `info.pose` (i.e. before that pose's own translation/rotation is
    applied) - see this module's docstring: row 0 sits at the max-x edge (x
    decreases as row increases), column 0 at the max-y edge (y decreases as column
    increases), matching grid_map's row-major/circular-buffer convention.
    """
    row_i = np.arange(rows, dtype=np.float32)
    col_j = np.arange(cols, dtype=np.float32)
    x = (length_x / 2.0) - (row_i + 0.5) * resolution
    y = (length_y / 2.0) - (col_j + 0.5) * resolution
    return np.meshgrid(x, y, indexing="ij")  # each (rows, cols)


def decode_grid_map(msg) -> DecodedGridMap:
    """Decodes a grid_map_msgs/GridMap message: unwraps every layer's circular
    buffer, picks the elevation layer ("elevation" if present, else the first entry
    of `basic_layers`, else the first entry of `layers`), builds its validity mask,
    and decodes a packed "color" layer if present.
    """
    layers = list(msg.layers)
    basic_layers = list(getattr(msg, "basic_layers", None) or [])
    outer_start = int(getattr(msg, "outer_start_index", 0))
    inner_start = int(getattr(msg, "inner_start_index", 0))

    layer_arrays: dict[str, np.ndarray] = {}
    rows = cols = 0
    for name, array in zip(layers, msg.data):
        dims = array.layout.dim
        r, c = int(dims[0].size), int(dims[1].size)
        rows, cols = r, c
        layer_arrays[name] = _unwrap_layer(array.data, r, c, outer_start, inner_start)

    if "elevation" in layer_arrays:
        elevation_layer = "elevation"
    elif basic_layers and basic_layers[0] in layer_arrays:
        elevation_layer = basic_layers[0]
    elif layers:
        elevation_layer = layers[0]
    else:
        elevation_layer = None

    elevation = (layer_arrays[elevation_layer] if elevation_layer is not None
                 else np.zeros((rows, cols), dtype=np.float32))

    valid = np.isfinite(elevation)
    validity_layers = basic_layers if basic_layers else ([elevation_layer] if elevation_layer else [])
    for name in validity_layers:
        if name in layer_arrays:
            valid = valid & np.isfinite(layer_arrays[name])

    rgb = None
    rgb_is_real = False
    if "color" in layer_arrays:
        rgb = _pack_color_layer(layer_arrays["color"])
        rgb_is_real = True

    extra_layers = {name: values for name, values in layer_arrays.items()
                     if name != elevation_layer and name != "color"}

    pose = msg.info.pose
    position = (float(pose.position.x), float(pose.position.y), float(pose.position.z))
    q = pose.orientation
    orientation = (float(q.x), float(q.y), float(q.z), float(q.w))

    return DecodedGridMap(
        rows=rows, cols=cols, resolution=float(msg.info.resolution),
        length_x=float(msg.info.length_x), length_y=float(msg.info.length_y),
        position=position, orientation=orientation,
        elevation=elevation.astype(np.float32), valid=valid,
        rgb=rgb, rgb_is_real=rgb_is_real, extra_layers=extra_layers,
    )
