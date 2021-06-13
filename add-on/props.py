import bpy


def GetBoneList(self, context):
    bones = []

    obj = context.active_object
    i = 0

    for bone in obj.pose.bones:
        bones.append((bone.name, bone.name, bone.name, i))
        i += 1

    return bones


class BoneAssignSettings(bpy.types.PropertyGroup):
    value: bpy.props.EnumProperty(items=GetBoneList, name="Bone List", description="A list of bones")
    name: bpy.props.StringProperty(name="AssignedBone")


class POSEFCAM_UL_BoneList(bpy.types.UIList):
    # @classmethod
    # def poll(cls, context):
    #    return (context.scene.source_action != "" and context.scene.source_rig != "" and context.scene.target_rig != "")

    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        split = layout.split(factor=1.0)
        split.prop(item, "name", text="", emboss=False, translate=False)  # icon='BONE_DATA')
        split = layout.split(factor=1.0)
        split.prop(item, "value", text="", emboss=False, translate=False)  # icon='BONE_DATA')

    def invoke(self, context, event):
        pass


def register():
    bpy.utils.register_class(BoneAssignSettings)
    bpy.utils.register_class(POSEFCAM_UL_BoneList)

    bpy.types.Object.BoneMappingSettings = bpy.props.CollectionProperty(type=BoneAssignSettings)
    bpy.types.Object.BoneActiveIndex = bpy.props.IntProperty()
    bpy.types.Scene.starting_keyframe = bpy.props.IntProperty(name="Start Keyframe",
                                                              description="Insert keyframes starting from this")
    bpy.types.Scene.keyframe_interval = bpy.props.IntProperty(name="Keyframe interval",
                                                              description="The interval between placed keyframes",
                                                              default=1)
    bpy.types.Scene.video_file_path = bpy.props.StringProperty(name="File Path", subtype='FILE_PATH', default="")


def unregister():
    bpy.utils.unregister_class(BoneAssignSettings)
    bpy.utils.unregister_class(POSEFCAM_UL_BoneList)

    del bpy.types.Object.BoneMappingSettings
    del bpy.types.Object.BoneActiveIndex
    del bpy.types.Scene.starting_keyframe
    del bpy.types.Scene.keyframe_interval
    del bpy.types.Scene.video_file_path
