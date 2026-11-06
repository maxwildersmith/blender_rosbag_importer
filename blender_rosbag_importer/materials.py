"""Shared materials for imported point clouds and grid maps: both are just a
Principled BSDF driven by the object's per-point/per-vertex "Color" attribute, so
they share one builder (`_ensure_attribute_color_material`) and differ only in name
(kept distinct so the outliner/material list reads sensibly) and which GN modifier
last wrote "Color":

- Point clouds: pointcloud2_decode.py bakes "Color" at import time from either the
  cloud's real RGB field or a grayscale intensity fallback, then
  point_cloud_nodes.py's crossfade modifier's live "Color Field" switch (real RGB, or
  any imported per-point scalar field run through a Color Ramp) overrides it at
  evaluation time.
- Grid maps: import_grid_map.py bakes "Color" from a packed grid_map "color" layer
  when one exists, then grid_map_nodes.py's modifier's "Use Elevation Color" switch
  (that packed color vs. a rainbow Color Ramp over `elevation`) overrides it the
  same way.

Either way, what a viewport/render actually sees is whatever the object's GN
modifier last wrote to "Color", since modifiers evaluate before shading - no change
needed here to support either live switch.
"""
import bpy

POINT_CLOUD_MATERIAL_NAME = "ROSBag PointCloud"
GRID_MAP_MATERIAL_NAME = "ROSBag GridMap"


def _ensure_attribute_color_material(name: str) -> bpy.types.Material:
    existing = bpy.data.materials.get(name)
    if existing is not None:
        return existing

    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()

    attr = nodes.new("ShaderNodeAttribute")
    attr.attribute_name = "Color"
    attr.attribute_type = "GEOMETRY"
    attr.location = (-300, 0)

    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.location = (0, 0)

    output = nodes.new("ShaderNodeOutputMaterial")
    output.location = (300, 0)

    links.new(attr.outputs["Color"], bsdf.inputs["Base Color"])
    links.new(bsdf.outputs["BSDF"], output.inputs["Surface"])

    return mat


def ensure_point_cloud_material() -> bpy.types.Material:
    """Returns the shared point-cloud material, creating it on first use."""
    return _ensure_attribute_color_material(POINT_CLOUD_MATERIAL_NAME)


def ensure_grid_map_material() -> bpy.types.Material:
    """Returns the shared grid-map material, creating it on first use."""
    return _ensure_attribute_color_material(GRID_MAP_MATERIAL_NAME)
