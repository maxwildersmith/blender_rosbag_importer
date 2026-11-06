"""Geometry Nodes modifier shared by every imported GridMap topic - the grid_map
counterpart to point_cloud_nodes.py, built the same way and for largely the same
reasons (see that module's docstring for background on why this add-on drives
per-object appearance through a live GN modifier rather than baking a fixed look in
at import time). It does two things:

Holes for invalid cells
-------------------------
import_grid_map.py builds the full rows x cols grid of vertices and quads ONCE, at
import time, and every frame just rewrites per-vertex attributes in place (mirroring
import_point_clouds.py) - including a `_valid` attribute (1.0 = real data, 0.0 =
NaN/unmapped cell). This modifier's Delete Geometry (domain POINT) removes every
invalid vertex - which, in Blender's mesh model, also removes every face that used
it - so unmapped cells (and their surrounding quads) become actual gaps in the
surface rather than a z=0 plateau or an arbitrary color. Unlike point_cloud_nodes.py's
crossfade, there is no dissolve here: GridMap scans on the same topic share the exact
same vertex grid (same rows/cols laid out the same way in space) from one message to
the next, so there's a real, stable per-vertex correspondence between scans - a
literal position (Z) lerp WOULD be meaningful here, unlike point clouds. This wasn't
asked for, so it isn't implemented (each scan just replaces the previous one
outright, the same "hold most recent scan" approach point clouds used before their
crossfade modifier existed) - see README for this noted as a possible enhancement.

Color source switch
--------------------
`Use Elevation Color` (boolean) switches the displayed "Color" attribute between a
real packed color layer (grid_map's "color" layer, baked in at import the same way
pointcloud2_decode.py bakes real RGB) and the `elevation` attribute (always present)
remapped by `Elevation Min`/`Elevation Max` into 0-1 and fed through a Color Ramp.
All four inputs are per-object modifier overrides despite one shared node group,
same as point_cloud_nodes.py's inputs - though that module's color-source switch has
since been generalized from a boolean into a String field-name input (any imported
per-point attribute, not just `intensity`); a boolean was kept here since a grid map
only ever has one candidate non-color field (`elevation`) to switch to, unlike point
clouds which can carry several (`intensity`, `ring`, `time`, ...).

Matching RViz's default look
------------------------------
`grid_map_rviz_plugin`'s default height coloring is a blue(low)-to-red(high) rainbow.
This is approximated (not claimed pixel-identical - the task only asked for a
"reasonable, clearly documented equivalent") with the same explicit 5-stop RGB
Color Ramp (blue -> cyan -> green -> yellow -> red) that point_cloud_nodes.py uses
for its per-field coloring - see color_ramps.py, shared by both so the two features
read as one visual language. Faces are left flat-shaded (not Shade Smooth) since
RViz's own per-cell quads read as flat-shaded too; a human can toggle Object > Shade
Smooth afterward if a smoother terrain look is preferred.
"""
import bpy

from .color_ramps import build_rainbow_ramp

NODE_GROUP_NAME = "ROSBag GridMap Coloring"
MODIFIER_NAME = "ROSBag GridMap Coloring"


def _iface_id(tree: bpy.types.GeometryNodeTree, name: str) -> str:
    """See point_cloud_nodes.py's `_iface_id` - identical helper, duplicated rather
    than shared since these two modules otherwise have no coupling."""
    for item in tree.interface.items_tree:
        if item.name == name:
            return item.identifier
    raise KeyError(f"No interface socket named {name!r} on {tree.name!r}")


def _new_input(tree, name, socket_type, default=None, min_value=None, max_value=None):
    socket = tree.interface.new_socket(name=name, in_out="INPUT", socket_type=socket_type)
    if default is not None:
        socket.default_value = default
    if min_value is not None:
        socket.min_value = min_value
    if max_value is not None:
        socket.max_value = max_value
    return socket


def _build_node_group() -> bpy.types.GeometryNodeTree:
    tree = bpy.data.node_groups.new(NODE_GROUP_NAME, "GeometryNodeTree")

    tree.interface.new_socket(name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    tree.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    _new_input(tree, "Use Elevation Color", "NodeSocketBool", default=False)
    _new_input(tree, "Elevation Min", "NodeSocketFloat", default=0.0)
    _new_input(tree, "Elevation Max", "NodeSocketFloat", default=1.0)

    nodes = tree.nodes
    links = tree.links
    nodes.clear()

    group_in = nodes.new("NodeGroupInput")
    group_in.location = (-800, 0)
    group_out = nodes.new("NodeGroupOutput")
    group_out.location = (700, 0)

    # --- delete invalid cells (and, as a side effect, every quad touching one) ---
    attr_valid = nodes.new("GeometryNodeInputNamedAttribute")
    attr_valid.data_type = "FLOAT"
    attr_valid.inputs["Name"].default_value = "_valid"
    attr_valid.location = (-800, -200)

    is_invalid = nodes.new("ShaderNodeMath")
    is_invalid.operation = "LESS_THAN"
    is_invalid.label = "_valid < 0.5 (delete)"
    is_invalid.inputs[1].default_value = 0.5
    is_invalid.location = (-550, -200)
    links.new(attr_valid.outputs[0], is_invalid.inputs[0])

    delete_geo = nodes.new("GeometryNodeDeleteGeometry")
    delete_geo.domain = "POINT"
    delete_geo.location = (-300, -60)
    links.new(group_in.outputs["Geometry"], delete_geo.inputs["Geometry"])
    # Implicit Float -> Boolean conversion: nonzero (1.0, i.e. invalid) deletes.
    links.new(is_invalid.outputs[0], delete_geo.inputs["Selection"])

    # --- color source switch: packed color vs. elevation through a rainbow ramp ---
    attr_color = nodes.new("GeometryNodeInputNamedAttribute")
    attr_color.data_type = "FLOAT_COLOR"
    attr_color.inputs["Name"].default_value = "Color"
    attr_color.location = (-800, 120)

    attr_elevation = nodes.new("GeometryNodeInputNamedAttribute")
    attr_elevation.data_type = "FLOAT"
    attr_elevation.inputs["Name"].default_value = "elevation"
    attr_elevation.location = (-800, 280)

    map_range = nodes.new("ShaderNodeMapRange")
    map_range.location = (-550, 280)
    links.new(attr_elevation.outputs[0], map_range.inputs[0])  # Value
    links.new(group_in.outputs["Elevation Min"], map_range.inputs[1])  # From Min
    links.new(group_in.outputs["Elevation Max"], map_range.inputs[2])  # From Max
    # To Min/To Max stay at their stock defaults (0.0/1.0) and Clamp stays on.

    ramp = nodes.new("ShaderNodeValToRGB")
    ramp.location = (-300, 280)
    build_rainbow_ramp(ramp)
    links.new(map_range.outputs[0], ramp.inputs[0])

    color_switch = nodes.new("GeometryNodeSwitch")
    color_switch.input_type = "RGBA"
    color_switch.label = "Use Elevation Color ?"
    color_switch.location = (-50, 180)
    links.new(group_in.outputs["Use Elevation Color"], color_switch.inputs[0])  # Switch
    links.new(attr_color.outputs[0], color_switch.inputs[1])  # False -> packed color
    links.new(ramp.outputs["Color"], color_switch.inputs[2])  # True -> elevation ramp

    store_color = nodes.new("GeometryNodeStoreNamedAttribute")
    store_color.domain = "POINT"
    store_color.data_type = "FLOAT_COLOR"
    store_color.inputs["Name"].default_value = "Color"
    store_color.location = (200, -60)
    links.new(delete_geo.outputs["Geometry"], store_color.inputs["Geometry"])
    links.new(color_switch.outputs[0], store_color.inputs["Value"])

    links.new(store_color.outputs["Geometry"], group_out.inputs["Geometry"])

    return tree


def ensure_grid_map_node_group() -> bpy.types.GeometryNodeTree:
    existing = bpy.data.node_groups.get(NODE_GROUP_NAME)
    if existing is not None:
        return existing
    return _build_node_group()


def ensure_grid_map_modifier(obj: bpy.types.Object, default_use_elevation: bool,
                              elevation_range: tuple | None) -> bpy.types.NodesModifier:
    """Adds the shared GridMap coloring/holes GN modifier to `obj` (if not already
    present), defaulting its per-object inputs from what this particular topic's
    data actually looks like - mirrors point_cloud_nodes.ensure_crossfade_modifier.
    `default_use_elevation` should be True when the topic never published a real
    "color" layer (CachedGridMapTopic.has_real_color is False), and `elevation_range`
    should be that topic's (min, max) elevation over valid cells
    (CachedGridMapTopic.elevation_range), or None if it had no valid cells at all.
    """
    existing = obj.modifiers.get(MODIFIER_NAME)
    if existing is not None:
        return existing

    tree = ensure_grid_map_node_group()
    mod = obj.modifiers.new(MODIFIER_NAME, "NODES")
    mod.node_group = tree

    mod[_iface_id(tree, "Use Elevation Color")] = bool(default_use_elevation)
    if elevation_range is not None:
        lo, hi = elevation_range
        mod[_iface_id(tree, "Elevation Min")] = float(lo)
        mod[_iface_id(tree, "Elevation Max")] = float(hi) if hi > lo else float(lo) + 1.0
    return mod
