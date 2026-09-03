import bpy


class ROSBAG_UL_topics(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        row = layout.row(align=True)
        row.prop(item, "selected", text="")
        row.label(text=item.name)
        row.label(text=item.msgtype)
        row.label(text=str(item.count))
        row.prop(item, "kind", text="")


CLASSES = (ROSBAG_UL_topics,)


def register() -> None:
    for cls in CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
