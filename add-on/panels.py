import bpy


class PoseMapPanel(bpy.types.Panel):
    """Creates a Panel in the Object properties window"""
    bl_label = "3D Pose Map Panel"
    bl_idname = "OBJECTDATA_PT_layout"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "data"

    def draw(self, context):
        layout = self.layout
        scene = context.scene

        # Create a simple row.
        layout.label(text="Video:")
        row = layout.row()

        # todo
        row = layout.row()
        row.prop_search(scene, "video_file_path", bpy.data, "objects", text="")

        row = layout.row()
        # Start frame and frame interval
        row.prop(scene, "keyframe_interval")
        row.prop(scene, "starting_keyframe")

        # Big render button
        layout.label(text="Big Button:")

        row = layout.row()
        row.operator("pose.initialize_map_operator", icon="OUTLINER_OB_ARMATURE")

        row = layout.row()
        row.scale_y = 3.0
        row.operator("pose.map_operator", icon="PLAY")

        row = layout.row()
        row.scale_y = 2.0
        row.operator("pose.sphere_test", icon="SHADING_SOLID")


class BoneAssignPanel(bpy.types.Panel):
    """Creates a Panel in the Object properties window"""
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "POSEFCAM Remap"
    bl_label = "Remapping Bones"
    bl_idname = "POSEFCAM_PT_assign_bones"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        obj = context.active_object

        row = layout.row(align=True)
        split = row.split(factor=0.5)
        split.label(text="Source Bones:")
        split.label(text="Target Bones:")

        row = layout.row(align=True)
        row.template_list("POSEFCAM_UL_BoneList", "", obj, "BoneMappingSettings", obj, "BoneActiveIndex", rows=2)

        layout.operator("pose.tweak_operator")
        layout.operator("pose.tweak_y")


def register():
    bpy.utils.register_class(PoseMapPanel)
    bpy.utils.register_class(BoneAssignPanel)


def unregister():
    bpy.utils.unregister_class(PoseMapPanel)
    bpy.utils.unregister_class(BoneAssignPanel)
