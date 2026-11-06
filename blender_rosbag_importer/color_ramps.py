"""Shared Color Ramp helper: the blue -> cyan -> green -> yellow -> red rainbow used
by both point_cloud_nodes.py (per-point scalar field coloring) and grid_map_nodes.py
(elevation coloring), so the two features read as one visual language instead of
each rolling a similar-but-different gradient. Approximates (not claimed
pixel-identical) RViz's default low-to-high rainbow coloring - an explicit 5-stop
RGB ramp is used rather than relying on a ColorRamp's HSV hue-interpolation
direction settings, which couldn't be verified against a live Blender session.
"""
import bpy

RAINBOW_STOPS = (
    (0.0, (0.0, 0.0, 1.0, 1.0)),   # low -> blue
    (0.25, (0.0, 1.0, 1.0, 1.0)),  # cyan
    (0.5, (0.0, 1.0, 0.0, 1.0)),   # green
    (0.75, (1.0, 1.0, 0.0, 1.0)),  # yellow
    (1.0, (1.0, 0.0, 0.0, 1.0)),   # high -> red
)


def build_rainbow_ramp(ramp_node: bpy.types.ShaderNodeValToRGB) -> None:
    """Configures a freshly-created Color Ramp node's stops in place to the 5-stop
    rainbow above. A fresh ColorRamp starts with exactly 2 elements (positions 0 and
    1); those are reused for the first/last stops and the rest are added.
    """
    ramp = ramp_node.color_ramp
    ramp.color_mode = "RGB"
    elements = ramp.elements
    elements[0].position, elements[0].color = RAINBOW_STOPS[0]
    elements[-1].position, elements[-1].color = RAINBOW_STOPS[-1]
    for position, color in RAINBOW_STOPS[1:-1]:
        element = elements.new(position)
        element.color = color
