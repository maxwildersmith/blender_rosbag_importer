"""Geometry Nodes modifier shared by every imported point-cloud topic: crossfades
between the two scans that import_point_clouds.py keeps loaded into the object's
Point Cloud data at any given moment, and lets the displayed color be switched live
- between real RGB and any imported per-point scalar field (`intensity`, `ring`,
`time`, ...) run through a rainbow Color Ramp - without re-importing anything.

Why a per-point dissolve instead of a position lerp
----------------------------------------------------
Consecutive PointCloud2 scans on a topic generally have different point counts and
no stable per-point correspondence - ROS doesn't attach persistent point IDs, so
"point i" of one scan and "point i" of the next are unrelated samples. Lerping
positions between them would slide essentially-random points across the scene
instead of tracking any real surface, which would look worse than a hard cut, not
better. So this crossfades geometrically instead: import_point_clouds.py writes
BOTH the outgoing scan (`_pc_src` = 0) and the incoming scan (`_pc_src` = 1) into the
same Point Cloud at once, each point also carrying a stable per-point random value
`_pc_rand` in [0, 1) (re-seeded from the pair's index so scrubbing back and forth
over the same pair always reproduces the same dissolve pattern rather than
flickering). This modifier's `Blend Factor` input - written every frame by
import_point_clouds.py's frame_change_pre handler, smoothstep-eased between the two
scans' frames - drives a Delete Geometry: outgoing points with `_pc_rand < factor`
are deleted, incoming points with `_pc_rand >= factor` are deleted. At factor 0 only
the outgoing scan is visible; at factor 1 only the incoming scan is; in between, a
growing random fraction of one has been swapped for the other. This is a stochastic
density/opacity-style crossfade (the documented fallback the task allowed), not a
true per-point motion blend - individual points still pop in/out rather than
sliding - but it reads as a smooth dissolve rather than an instant snap, needs no
point correspondence, and is a real geometry-level effect (Delete Geometry actually
removes points at evaluation time) rather than a shader trick.

Known tradeoff: for the whole span between two scans' frames (not just a brief
transition window at the boundary), the base Point Cloud holds BOTH scans' points
at once, so the point count - and per-point attribute memory - roughly doubles for
the duration of every inter-scan segment. For very dense scans or a very large
`Max Points/Scan` this is a real cost; lowering that import setting is the mitigation
if playback gets choppy.

Color source switch
--------------------
`Color Field` (a String modifier input) picks which per-point attribute drives the
point cloud's displayed "Color": empty ("") means the baseline - the real per-point
RGB baked in at import by pointcloud2_decode.py (or its grayscale-from-intensity
fallback, or nothing, whatever ended up in the "Color" attribute) - any other value
names an arbitrary per-point scalar attribute that import_point_clouds.py wrote one
FLOAT attribute for per extra PointCloud2 field (e.g. "intensity", "ring", "time" -
whatever a given topic's messages actually carried), which gets remapped by `Field
Min`/`Field Max` into 0-1 and fed through the same blue-cyan-green-yellow-red
rainbow Color Ramp grid_map_nodes.py uses for elevation (see color_ramps.py) - one
shared visual language for both features rather than two different-looking ramps.
A `GeometryNodeInputNamedAttribute` node reads whatever name `Color Field` currently
holds, so typing any attribute name in - not just a name this add-on anticipated -
works without editing the node tree; a name with no matching attribute just reads
as a flat default value (0.0), which through the ramp shows as a flat color rather
than an error, so a typo is easy to notice (nothing looks like it's supposed to) but
never breaks anything.

All inputs (Blend Factor, Color Field, Field Min, Field Max) are per-object modifier
overrides despite one shared node group - so each topic's field/range can be set
independently and edited live from the modifier's panel in the Properties editor,
no re-import needed. import_point_clouds.py defaults `Color Field` per topic to
"intensity" only when BOTH the topic never published a real color field
(point_cloud_cache.CachedPointCloudTopic.has_real_rgb is False) AND it actually has
an `intensity` field (rather than blindly assuming one exists, which the previous
boolean-switch version did) - otherwise it defaults to "" (the RGB/baseline
behavior), preserving "real RGB wins by default when a topic has one" exactly as
before. Min/Max seed from that field's topic-wide observed range
(CachedPointCloudTopic.field_ranges). If a human retypes `Color Field` to name a
different attribute (e.g. "ring"), Min/Max do NOT auto-update to match it (there's
no live "recompute the range of whatever's currently in Color Field" node in this
tree, on purpose - see below) - they're just left at whatever they were, so the
ramp may look wrong (over- or under-saturated) until Min/Max are adjusted by hand to
that attribute's real range.

**Why Min/Max isn't computed live instead:** Blender does have a
`GeometryNodeAttributeStatistic` node that could read Min/Max off of whatever
attribute `Color Field` currently names, live, with no Python-side range tracking
needed at all - which would sidestep the stale-range problem entirely. This wasn't
used because the task this was built for explicitly asked to generalize the
existing Python-side range-tracking approach (point_cloud_cache.py's per-field
`field_ranges`) rather than replace it; that's a reasonable follow-up if the
stale-range tradeoff above proves annoying in practice.

The Color Ramp itself (its color stops) is a node-internal datablock, not a
modifier input socket - Blender has no "ramp" socket type - so it isn't exposed in
the modifier panel and is shared across every point-cloud topic (like the shared
material in materials.py). To restyle it, open a Geometry Nodes editor, select any
imported point cloud object, and edit the "Color Ramp" node's stops directly; this
affects every topic using a non-baseline `Color Field`, not just one.
color_ramps.py's `build_rainbow_ramp` only sets a ramp's stops once, at node
creation time - this node group's own Color Ramp node is a separate instance from
grid_map_nodes.py's, so restyling one doesn't touch the other even though both
started out identical.
"""
import bpy

from .color_ramps import build_rainbow_ramp

NODE_GROUP_NAME = "ROSBag PointCloud Crossfade"
MODIFIER_NAME = "ROSBag Crossfade"


def _iface_id(tree: bpy.types.GeometryNodeTree, name: str) -> str:
    """Looks up an interface socket's `identifier` (what modifier inputs are keyed
    by) from its human-readable `name`. A small linear scan over a handful of
    sockets, cheap enough to redo on every call - including every frame from
    set_blend_factor() - rather than caching it against the interface changing.
    """
    for item in tree.interface.items_tree:
        if item.name == name:
            return item.identifier
    raise KeyError(f"No interface socket named {name!r} on {tree.name!r}")


def _new_input(tree: bpy.types.GeometryNodeTree, name: str, socket_type: str, default=None,
               min_value=None, max_value=None) -> bpy.types.NodeTreeInterfaceSocket:
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
    _new_input(tree, "Blend Factor", "NodeSocketFloat", default=0.0, min_value=0.0, max_value=1.0)
    _new_input(tree, "Color Field", "NodeSocketString", default="")
    _new_input(tree, "Field Min", "NodeSocketFloat", default=0.0)
    _new_input(tree, "Field Max", "NodeSocketFloat", default=1.0)

    nodes = tree.nodes
    links = tree.links
    nodes.clear()

    group_in = nodes.new("NodeGroupInput")
    group_in.location = (-900, 0)
    group_out = nodes.new("NodeGroupOutput")
    group_out.location = (900, 0)

    # --- per-point dissolve selection: delete = XNOR(is_outgoing, rand < factor) ---
    # implemented purely with single-purpose Math nodes (unambiguous input/output
    # socket indices) rather than Compare/Boolean Math, to avoid the multi-datatype
    # socket-index ambiguity those consolidated nodes have.
    attr_src = nodes.new("GeometryNodeInputNamedAttribute")
    attr_src.data_type = "FLOAT"
    attr_src.inputs["Name"].default_value = "_pc_src"
    attr_src.location = (-900, -220)

    attr_rand = nodes.new("GeometryNodeInputNamedAttribute")
    attr_rand.data_type = "FLOAT"
    attr_rand.inputs["Name"].default_value = "_pc_rand"
    attr_rand.location = (-900, -380)

    is_outgoing = nodes.new("ShaderNodeMath")
    is_outgoing.operation = "LESS_THAN"
    is_outgoing.label = "src < 0.5 (is outgoing)"
    is_outgoing.inputs[1].default_value = 0.5
    is_outgoing.location = (-650, -220)
    links.new(attr_src.outputs[0], is_outgoing.inputs[0])

    rand_lt_factor = nodes.new("ShaderNodeMath")
    rand_lt_factor.operation = "LESS_THAN"
    rand_lt_factor.label = "rand < Blend Factor"
    rand_lt_factor.location = (-650, -380)
    links.new(attr_rand.outputs[0], rand_lt_factor.inputs[0])
    links.new(group_in.outputs["Blend Factor"], rand_lt_factor.inputs[1])

    diff = nodes.new("ShaderNodeMath")
    diff.operation = "SUBTRACT"
    diff.location = (-420, -300)
    links.new(is_outgoing.outputs[0], diff.inputs[0])
    links.new(rand_lt_factor.outputs[0], diff.inputs[1])

    abs_diff = nodes.new("ShaderNodeMath")
    abs_diff.operation = "ABSOLUTE"
    abs_diff.location = (-200, -300)
    links.new(diff.outputs[0], abs_diff.inputs[0])

    delete_flag = nodes.new("ShaderNodeMath")
    delete_flag.operation = "SUBTRACT"
    delete_flag.label = "delete = XNOR(...)"
    delete_flag.inputs[0].default_value = 1.0
    delete_flag.location = (30, -300)
    links.new(abs_diff.outputs[0], delete_flag.inputs[1])

    delete_geo = nodes.new("GeometryNodeDeleteGeometry")
    delete_geo.domain = "POINT"
    delete_geo.location = (250, -120)
    links.new(group_in.outputs["Geometry"], delete_geo.inputs["Geometry"])
    # Implicit Float -> Boolean conversion: nonzero (1.0) selects for deletion.
    links.new(delete_flag.outputs[0], delete_geo.inputs["Selection"])

    # --- color source switch: RGB (baked "Color") vs. an arbitrary named field ---
    attr_color = nodes.new("GeometryNodeInputNamedAttribute")
    attr_color.data_type = "FLOAT_COLOR"
    attr_color.inputs["Name"].default_value = "Color"
    attr_color.location = (-900, 120)

    # Unlike attr_src/attr_rand/attr_color above (all fixed, hardcoded attribute
    # names), this Named Attribute's "Name" is LINKED to the Color Field group
    # input instead of given a fixed default_value - so whatever attribute name a
    # human types into the modifier's Color Field box is what gets read here,
    # without this node tree needing to know it in advance. A name that doesn't
    # match any attribute just reads as 0.0 for every point (Blender's documented
    # behavior for a missing Named Attribute), which through the ramp below just
    # shows as a flat, obviously-wrong color rather than an error.
    attr_field = nodes.new("GeometryNodeInputNamedAttribute")
    attr_field.data_type = "FLOAT"
    attr_field.label = "Color Field value"
    attr_field.location = (-900, 280)
    links.new(group_in.outputs["Color Field"], attr_field.inputs["Name"])

    map_range = nodes.new("ShaderNodeMapRange")
    map_range.location = (-650, 280)
    links.new(attr_field.outputs[0], map_range.inputs[0])  # Value
    links.new(group_in.outputs["Field Min"], map_range.inputs[1])  # From Min
    links.new(group_in.outputs["Field Max"], map_range.inputs[2])  # From Max
    # To Min/To Max are left at their stock defaults (0.0/1.0) and Clamp stays on,
    # so anything outside [Min, Max] just clamps to the ramp's end stops.

    ramp = nodes.new("ShaderNodeValToRGB")
    ramp.location = (-420, 280)
    build_rainbow_ramp(ramp)
    links.new(map_range.outputs[0], ramp.inputs[0])

    # "Use a field at all?" is just "is Color Field non-empty?" - String Length
    # returns 0 for "" and >0 for anything else, and that Int implicitly converts
    # to the Switch node's Boolean input (nonzero = True) with no extra node needed.
    field_name_length = nodes.new("FunctionNodeStringLength")
    field_name_length.location = (-650, 450)
    links.new(group_in.outputs["Color Field"], field_name_length.inputs["String"])

    color_switch = nodes.new("GeometryNodeSwitch")
    color_switch.input_type = "RGBA"
    color_switch.label = "Color Field set ?"
    color_switch.location = (250, 180)
    # Implicit Int -> Boolean conversion: a nonzero name length switches to the field.
    links.new(field_name_length.outputs[0], color_switch.inputs[0])  # Switch
    links.new(attr_color.outputs[0], color_switch.inputs[1])  # False -> real/baked RGB
    links.new(ramp.outputs["Color"], color_switch.inputs[2])  # True -> Color Field ramp

    store_color = nodes.new("GeometryNodeStoreNamedAttribute")
    store_color.domain = "POINT"
    store_color.data_type = "FLOAT_COLOR"
    store_color.inputs["Name"].default_value = "Color"
    store_color.location = (480, -60)
    links.new(delete_geo.outputs["Geometry"], store_color.inputs["Geometry"])
    links.new(color_switch.outputs[0], store_color.inputs["Value"])

    links.new(store_color.outputs["Geometry"], group_out.inputs["Geometry"])

    return tree


def ensure_crossfade_node_group() -> bpy.types.GeometryNodeTree:
    """Returns the shared crossfade node group, building it on first use (mirrors
    materials.ensure_point_cloud_material's shared-datablock pattern).
    """
    existing = bpy.data.node_groups.get(NODE_GROUP_NAME)
    if existing is not None:
        return existing
    return _build_node_group()


def ensure_crossfade_modifier(obj: bpy.types.Object, default_color_field: str,
                               default_field_range: tuple | None) -> bpy.types.NodesModifier:
    """Adds the shared crossfade GN modifier to `obj` (if not already present),
    defaulting its per-object inputs from what this particular topic's data actually
    looks like. `default_color_field` should be "" (the RGB/baseline "Color"
    attribute) unless the topic has no real color field AND does have an
    `intensity` field, in which case it should be "intensity" - see
    import_point_clouds.py's call site for exactly that decision, which preserves
    "real RGB wins by default" from before this switch was generalized past a fixed
    intensity/RGB boolean. `default_field_range` should be that field's (min, max)
    (point_cloud_cache.CachedPointCloudTopic.field_ranges[default_color_field]), or
    None if `default_color_field` is "" or has no tracked range.
    """
    existing = obj.modifiers.get(MODIFIER_NAME)
    if existing is not None:
        return existing

    tree = ensure_crossfade_node_group()
    mod = obj.modifiers.new(MODIFIER_NAME, "NODES")
    mod.node_group = tree

    mod[_iface_id(tree, "Color Field")] = default_color_field or ""
    if default_field_range is not None:
        lo, hi = default_field_range
        mod[_iface_id(tree, "Field Min")] = float(lo)
        mod[_iface_id(tree, "Field Max")] = float(hi) if hi > lo else float(lo) + 1.0
    mod[_iface_id(tree, "Blend Factor")] = 0.0
    return mod


def set_blend_factor(obj: bpy.types.Object, factor: float) -> None:
    """Called every frame by import_point_clouds.py's frame_change_pre handler -
    there's no native Blender driver expression that could express "fraction of the
    way between this pair's two irregularly-spaced scan frames" without a custom
    driver function, and the handler already recomputes this every frame anyway, so
    this just writes the modifier input directly instead.
    """
    mod = obj.modifiers.get(MODIFIER_NAME)
    if mod is None:
        return
    mod[_iface_id(mod.node_group, "Blend Factor")] = factor
