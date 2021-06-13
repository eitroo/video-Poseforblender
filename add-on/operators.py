import numpy as np
import bpy
from mathutils import *
import math
import os
import VideoPose3D


def GetBoneList(context):
    bones = []

    obj = context.active_object
    i = 0

    for bone in obj.pose.bones:
        bones.append((bone.name, bone.name, bone.name, i))
        i += 1

    return bones


def InitializeBoneDictionaryAuto():
    obj = bpy.context.active_object
    boneKeys = ['Pelvis', 'Hip_L', 'Knee_L', 'Ankle_L',
                'Hip_R', 'Knee_R', 'Ankle_R', 'Spine1',
                'Neck', 'Head', 'Site', 'Shoulder_R',
                'Elbow_R', 'Wrist_R', 'Shoulder_L', 'Elbow_L', 'Wrist_L']

    boneList = GetBoneList(bpy.context)

    for bone in boneKeys:
        my_item = obj.BoneMappingSettings.add()
        my_item.name = bone

        found = False

        for tBone in boneList:
            name = tBone[0].lower()

            if bone == 'Pelvis' and ('hips' in name or 'pelvis' in name):
                my_item.value = tBone[0]
                found = True

            if bone == 'Hip_L' and (('hip' in name or ('leg' in name and 'up' in name)) and
                                    ('left' in name or 'L' in tBone[0])):
                my_item.value = tBone[0]
                found = True

            if bone == 'Knee_L' and (('knee' in name or 'leg' in name) and ('left' in name or 'L' in tBone[0])):
                my_item.value = tBone[0]
                found = True

            if bone == 'Ankle_L' and (('ankle' in name or 'foot' in name) and ('left' in name or 'L' in tBone[0])):
                my_item.value = tBone[0]
                found = True

            if bone == 'Hip_R' and (('hip' in name or ('leg' in name and 'up' in name)) and
                                    ('right' in name or 'R' in tBone[0])):
                my_item.value = tBone[0]
                found = True

            if bone == 'Knee_R' and (('knee' in name or 'leg' in name) and ('right' in name or 'R' in tBone[0])):
                my_item.value = tBone[0]
                found = True

            if bone == 'Ankle_R' and (('ankle' in name or 'foot' in name) and ('right' in name or 'R' in tBone[0])):
                my_item.value = tBone[0]
                found = True

            if bone == 'Spine1' and ('spine1' in name):
                my_item.value = tBone[0]
                found = True

            if bone == 'Neck' and ('neck' in name):
                my_item.value = tBone[0]
                found = True

            if bone == 'Head' and ('head' in name):
                my_item.value = tBone[0]
                found = True

            if bone == 'Site' and ('site' in name or ('head' in name and 'top' in name)):
                my_item.value = tBone[0]
                found = True

            if bone == 'Shoulder_R' and (
                    ('shoulder' in name or 'arm' in name) and ('right' in name or 'R' in tBone[0])):
                my_item.value = tBone[0]
                found = True

            if bone == 'Elbow_R' and (('elbow' in name or 'forearm' in name) and ('right' in name or 'R' in tBone[0])):
                my_item.value = tBone[0]
                found = True

            if bone == 'Wrist_R' and (('wrist' in name or 'hand' in name) and ('right' in name or 'R' in tBone[0])):
                my_item.value = tBone[0]
                found = True

            if bone == 'Shoulder_L' and (('shoulder' in name or 'arm' in name) and ('left' in name or 'L' in tBone[0])):
                my_item.value = tBone[0]
                found = True

            if bone == 'Elbow_L' and (('elbow' in name or 'forearm' in name) and ('left' in name or 'L' in tBone[0])):
                my_item.value = tBone[0]
                found = True

            if bone == 'Wrist_L' and (('wrist' in name or 'hand' in name) and ('left' in name or 'L' in tBone[0])):
                my_item.value = tBone[0]
                found = True

            if found == True:
                break

        if found == False:
            my_item.value = boneList[0][0]


def InitializeBoneDictionaryMixamo():
    obj = bpy.context.active_object
    boneKeys = ['Pelvis', 'Hip_L', 'Knee_L', 'Ankle_L',
                'Hip_R', 'Knee_R', 'Ankle_R', 'Spine1',
                'Neck', 'Head', 'Site', 'Shoulder_R',
                'Elbow_R', 'Wrist_R', 'Shoulder_L', 'Elbow_L', 'Wrist_L']
    switcher = {
        'Pelvis': "mixamorig:Hips",
        'Hip_L': "mixamorig:LeftUpLeg",
        'Knee_L': "mixamorig:LeftLeg",
        'Ankle_L': "mixamorig:LeftFoot",
        'Hip_R': "mixamorig:RightUpLeg",
        'Knee_R': "mixamorig:RightLeg",
        'Ankle_R': "mixamorig:RightFoot",
        'Spine1': "mixamorig:Spine1",
        'Neck': "mixamorig:Neck",
        'Head': "mixamorig:Head",
        'Site': "mixamorig:HeadTop_End",
        'Shoulder_R': "mixamorig:RightArm",
        'Elbow_R': "mixamorig:RightForeArm",
        'Wrist_R': "mixamorig:RightHand",
        'Shoulder_L': "mixamorig:LeftArm",
        'Elbow_L': "mixamorig:LeftForeArm",
        'Wrist_L': "mixamorig:LeftHand"
    }
    for bone in boneKeys:
        my_item = obj.BoneMappingSettings.add()
        my_item.name = bone
        my_item.value = switcher.get(bone)


def IsMixamo(context):
    obj = bpy.context.active_object
    boneList = GetBoneList(bpy.context)
    for tBone in boneList:
        name = tBone[0].lower()
        if("mixamorig" in name):
            return True
    return False


def InferPose():
    # infer_video_d2
    if os.path.splitext(bpy.context.scene['video_file_path'])[1] != '.mp4':
        return

    VideoPose3D.Infer(bpy.context.scene['video_file_path'])


def MapPose():
    extension = os.path.splitext(bpy.context.scene['video_file_path'])[1]
    if extension == '.npy':
        readPositions = np.load(bpy.path.abspath(bpy.context.scene["video_file_path"]))
    elif extension == '.mp4':
        readPositions = np.load(VideoPose3D.GetOutputPath())
    else:
        return

    readPositions = ApplyRotationMatrix(readPositions)
    rotations = [0] * 17
    for i in range(0, len(readPositions)):
        bpy.ops.object.mode_set(mode='POSE')
        SwitchCoordinates(readPositions[i])
        rotations = CalculateRotations(readPositions[i], rotations)
        RotateBones(i, rotations)


def ApplyRotationMatrix(array4D):
    theta = np.radians(90)
    c, s = np.cos(theta), np.sin(theta)
    Rx = np.array(((1, 0, 0), (0, c, -s), (0, s, c)))
    for i in range(0, len(array4D)):
        array4D[i] = array4D[i].dot(Rx)
    return array4D


def CalculateRotations(frame, rotations):
    arm = bpy.context.active_object

    for i in range(0, 17):
        bone = arm.BoneMappingSettings[i].value

        if i not in [3, 6, 10, 13, 16]:
            vector1 = arm.pose.bones[bone].y_axis.copy()
            vector1.normalize()

            if (i == 0):
                vector2 = Vector(frame[7]) - Vector(frame[0])
                vector2.normalize()
                angleInRadians = vector1.angle(vector2)
                axis = vector1.cross(vector2)
                rotations[i] = Quaternion(axis, angleInRadians)

                hipNormalCurrent = arm.pose.bones[bone].z_axis.copy()
                hipNormalCurrent.rotate(rotations[i])

                hips = Vector(frame[4])
                hips.normalize()

                hipNormal = vector2.cross(hips)

                hipNormal.normalize()

                hipAngle = hipNormalCurrent.angle(hipNormal)
                hipAxis = hipNormalCurrent.cross(hipNormal)
                hipQuaternion = Quaternion(hipAxis, hipAngle)
                rotations[i].rotate(hipQuaternion)

            else:
                vector2 = Vector(frame[i + 1]) - Vector(frame[i])
                vector2.normalize()
                angleInRadians = vector1.angle(vector2)
                axis = vector1.cross(vector2)
                rotations[i] = Quaternion(axis, angleInRadians)

                if (i == 7):
                    shoulderNormalCurrent = arm.pose.bones[bone].z_axis.copy()
                    shoulderNormalCurrent.rotate(rotations[i])
                    shoulder = Vector(frame[11]) - Vector(frame[14]) #indexes are for left and right shoulders
                    shoulder.normalize()

                    shoulderNormal = vector2.cross(shoulder)

                    shoulderNormal.normalize()

                    shoulderAngle = shoulderNormalCurrent.angle(shoulderNormal)
                    shoulderAxis = shoulderNormalCurrent.cross(shoulderNormal)
                    shoulderQuaternion = Quaternion(shoulderAxis, shoulderAngle)
                    rotations[i].rotate(shoulderQuaternion)

        else:
            rotations[i] = Quaternion()

    return rotations


def RotateBones(frameIndex, rotations):
    arm = bpy.context.active_object
    boneKeys = ['Pelvis', 'Hip_L', 'Knee_L', 'Ankle_L',
                'Hip_R', 'Knee_R', 'Ankle_R', 'Spine1',
                'Neck', 'Head', 'Site', 'Shoulder_R',
                'Elbow_R', 'Wrist_R', 'Shoulder_L', 'Elbow_L', 'Wrist_L']

    for boneKey in boneKeys:
        if boneKey in ['Ankle_R', 'Ankle_L', 'Site', 'Wrist_R', 'Wrist_L']:
            continue

        bone = arm.BoneMappingSettings[boneKeys.index(boneKey)].value

        rot_mat = rotations[boneKeys.index(boneKey)].to_matrix().to_4x4()
        orig_loc, orig_rot, orig_scale = arm.pose.bones[bone].matrix.decompose()
        orig_loc_mat = Matrix.Translation(orig_loc)
        orig_rot_mat = orig_rot.to_matrix().to_4x4()
        orig_scale_mat = Matrix.Scale(orig_scale[0], 4, (1, 0, 0)) * \
                         Matrix.Scale(orig_scale[1], 4, (0, 1, 0)) * \
                         Matrix.Scale(orig_scale[2], 4, (0, 0, 1))

        arm.pose.bones[bone].matrix = orig_loc_mat @ rot_mat @ orig_rot_mat @ orig_scale_mat

        if frameIndex != 0 or frameIndex != 1:
            arm.pose.bones[bone].keyframe_insert('rotation_quaternion',
                                             frame=bpy.context.scene[
                                                       "starting_keyframe"] + ((frameIndex-2)*bpy.context.scene["keyframe_interval"])
                                                        if "starting_keyframe" in bpy.context.scene else frameIndex-2)


def SwitchCoordinates(array):
    array[:, [1, 2]] = array[:, [2, 1]]
    array[:, 2] = -array[:, 2]
    array[:, 1] = -array[:, 1]


def ApplyRolls(context, diff, exceptions):
    action_name = context.object.animation_data.action.name
    fcurves = bpy.data.actions[action_name].fcurves
    diffIndex = 0
    currentFrame = context.scene.frame_current

    for i in range(0, len(fcurves)):
        bone_name = fcurves[i].data_path.split('"')[1]
        if 'rotation_quaternion' in fcurves[i].data_path and \
                bone_name not in exceptions and fcurves[i].array_index == 0:

            if math.isclose(diff[diffIndex], 0.0):
                diffIndex += 1
                continue

            if (len(context.active_object.pose.bones[bone_name].children) == 1):
                for keyframePoint in fcurves[i].keyframe_points:
                    context.scene.frame_set(keyframePoint.co[0])
                    bone = context.active_object.pose.bones[bone_name]
                    bone.rotation_mode = 'XYZ'
                    bone.rotation_euler.rotate_axis('Y', diff[diffIndex])
                    bone.rotation_mode = 'QUATERNION'

                    bone.keyframe_insert('rotation_quaternion',
                                         frame=context.scene.frame_current,
                                         options={'INSERTKEY_REPLACE'})


            diffIndex += 1
    context.scene.frame_set(currentFrame)


def CalculateTweakDifferenceY(context):
    currentFrame = context.scene.frame_current
    action_name = context.object.animation_data.action.name
    fcurves = bpy.data.actions[action_name].fcurves
    diff = []
    exceptions = []

    for i in range(0, len(fcurves)):
        if 'rotation_quaternion' in fcurves[i].data_path and fcurves[i].array_index == 0:
            bone_name = fcurves[i].data_path.split('"')[1]
            try:
                w = fcurves[i].keyframe_points[currentFrame].co[1]
                x = fcurves[i + 1].keyframe_points[currentFrame].co[1]
                y = fcurves[i + 2].keyframe_points[currentFrame].co[1]
                z = fcurves[i + 3].keyframe_points[currentFrame].co[1]
                q = Quaternion([w, x, y, z])

                quatDiff = q.rotation_difference(context.active_object.pose.bones[bone_name].rotation_quaternion)

                axis, angle = quatDiff.to_axis_angle()
                print(f"Axis:{axis}, Angle:{angle}")

                if math.isclose(0.0, axis[0], abs_tol=1e-02) and \
                        (math.isclose(1.0, axis[1], rel_tol=1e-06) or math.isclose(-1.0, axis[1], rel_tol=1e-06)) and \
                        math.isclose(0.0, axis[2], abs_tol=1e-02):
                    print("axis aligned")
                    if (axis[1] > 0):
                        diff.append(angle)
                    else:
                        diff.append(-angle)
                else:
                    diff.append(0.0)
            except:
                exceptions.append(bone_name)
                pass

    print(diff)
    return diff, exceptions


def CalculateTweakDifference(context):
    currentFrame = context.scene.frame_current
    action_name = context.object.animation_data.action.name
    fcurves = bpy.data.actions[action_name].fcurves
    diff = []
    exceptions = []
    for i in range(0, len(fcurves)):
        if 'rotation_quaternion' in fcurves[i].data_path and fcurves[i].array_index == 0:
            bone_name = fcurves[i].data_path.split('"')[1]
            try:
                w = fcurves[i].keyframe_points[currentFrame].co[1]
                x = fcurves[i + 1].keyframe_points[currentFrame].co[1]
                y = fcurves[i + 2].keyframe_points[currentFrame].co[1]
                z = fcurves[i + 3].keyframe_points[currentFrame].co[1]
                q = Quaternion([w, x, y, z])
                # quatDiff = context.active_object.pose.bones[bone_name].rotation_quaternion.rotation_difference(q)
                quatDiff = context.active_object.pose.bones[bone_name].rotation_quaternion @ q.inverted()

                diff.append(quatDiff)
            except:
                exceptions.append(bone_name)
                pass
    return diff, exceptions


class InitializePoseMapOperator(bpy.types.Operator):
    bl_idname = "pose.initialize_map_operator"
    bl_label = "Initialize a bone dictionary"

    @classmethod
    def poll(cls, context):
        return context.active_object.type == 'ARMATURE' and context.active_object.BoneMappingSettings.values() == []

    def execute(self, context):
        if(IsMixamo(context)):
            InitializeBoneDictionaryMixamo()
        else:
            InitializeBoneDictionaryAuto()
        return {'FINISHED'}


class PoseMapOperator(bpy.types.Operator):
    """
    Mappings must be initialized before use.
    """
    bl_idname = "pose.map_operator"
    bl_label = "Estimate 3D pose from video"

    @classmethod
    def poll(cls, context):
        if "BoneMappingSettings" in context.active_object:
            if "keyframe_interval" in context.scene:
                if context.scene["keyframe_interval"] > 0:
                    return True

        return False

    def execute(self, context):
        InferPose()
        MapPose()
        return {'FINISHED'}


class SphereTestOperator(bpy.types.Operator):
    bl_idname = "pose.sphere_test"
    bl_label = "Animated spheres tracking the joints"

    def execute(self, context):

        boneKeys = ['Pelvis', 'Hip_L', 'Knee_L', 'Ankle_L',
                    'Hip_R', 'Knee_R', 'Ankle_R', 'Spine1',
                    'Neck', 'Head', 'Site', 'Shoulder_R',
                    'Elbow_R', 'Wrist_R', 'Shoulder_L', 'Elbow_L', 'Wrist_L']

        if os.path.splitext(bpy.context.scene['video_file_path'])[1] == '.npy':
            readPositions = np.load(bpy.path.abspath(bpy.context.scene["video_file_path"]))
        else:
            readPositions = np.load(VideoPose3D.GetOutputPath())

        for b in boneKeys:
            if "TrackingSphere_" + b not in bpy.context.scene.objects:
                bpy.ops.mesh.primitive_uv_sphere_add(radius=0.05, calc_uvs=True, enter_editmode=False, align='WORLD',
                                                     location=(0.0, 0.0, 0.0))
                bpy.context.active_object.name = "TrackingSphere_" + b

        for frameIndex in range(0, len(readPositions)):
            SwitchCoordinates(readPositions[frameIndex])
            for i in range(0, 17):
                bpy.data.objects['TrackingSphere_' + boneKeys[i]].location = readPositions[frameIndex][i]
                bpy.data.objects['TrackingSphere_' + boneKeys[i]].keyframe_insert('location',
                                                                                  frame=bpy.context.scene[
                                                                                            "starting_keyframe"] + frameIndex if "starting_keyframe" in bpy.context.scene else frameIndex)

        return {'FINISHED'}


class TweakOperator(bpy.types.Operator):
    """
    Apply tweak across the action
    """
    bl_idname = "pose.tweak_operator"
    bl_label = "Tweak"
    bl_options = {'UNDO'}

    @classmethod
    def poll(cls, context):
        if context.active_object.type == 'ARMATURE':
            # if "BoneMappingSettings" in context.active_object:
            #    return True

            if hasattr(context.active_object, "BoneMappingSettings"):
                return True
        return False

    def execute(self, context):
        action_name = context.object.animation_data.action.name
        fcurves = bpy.data.actions[action_name].fcurves
        diff, exceptions = CalculateTweakDifference(context)
        diffIndex = 0

        for i in range(0, len(fcurves)):
            bone_name = fcurves[i].data_path.split('"')[1]
            if 'rotation_quaternion' in fcurves[i].data_path and bone_name not in exceptions and fcurves[
                i].array_index == 0:
                for j in range(0, len(fcurves[i].keyframe_points)):
                    w = fcurves[i].keyframe_points[j].co[1]
                    x = fcurves[i + 1].keyframe_points[j].co[1]
                    y = fcurves[i + 2].keyframe_points[j].co[1]
                    z = fcurves[i + 3].keyframe_points[j].co[1]
                    Q = Quaternion([w, x, y, z])
                    result = diff[diffIndex] @ Q

                    fcurves[i].keyframe_points[j].co[1] = result.w
                    fcurves[i + 1].keyframe_points[j].co[1] = result.x
                    fcurves[i + 2].keyframe_points[j].co[1] = result.y
                    fcurves[i + 3].keyframe_points[j].co[1] = result.z

                    fcurves[i].keyframe_points[j].handle_left[1] = result.w
                    fcurves[i + 1].keyframe_points[j].handle_left[1] = result.x
                    fcurves[i + 2].keyframe_points[j].handle_left[1] = result.y
                    fcurves[i + 3].keyframe_points[j].handle_left[1] = result.z

                    fcurves[i].keyframe_points[j].handle_right[1] = result.w
                    fcurves[i + 1].keyframe_points[j].handle_right[1] = result.x
                    fcurves[i + 2].keyframe_points[j].handle_right[1] = result.y
                    fcurves[i + 3].keyframe_points[j].handle_right[1] = result.z

                diffIndex += 1

        context.scene.frame_set(context.scene.frame_current)
        return {'FINISHED'}


class RotateYOperator(bpy.types.Operator):
    """
    Apply a tweak rotation of bones in y axis.
    """
    bl_idname = "pose.tweak_y"
    bl_label = "Tweak Y"
    bl_options = {'UNDO'}

    @classmethod
    def poll(cls, context):
        if context.active_object.type == 'ARMATURE':
            # if "BoneMappingSettings" in context.active_object:
            #    return True

            if hasattr(context.active_object, "BoneMappingSettings"):
                return True
        return False

    def execute(self, context):
        diff, exceptions = CalculateTweakDifferenceY(context)
        ApplyRolls(context, diff, exceptions)

        return {'FINISHED'}


def register():
    bpy.utils.register_class(InitializePoseMapOperator)
    bpy.utils.register_class(PoseMapOperator)
    bpy.utils.register_class(SphereTestOperator)
    bpy.utils.register_class(TweakOperator)
    bpy.utils.register_class(RotateYOperator)


def unregister():
    bpy.utils.unregister_class(InitializePoseMapOperator)
    bpy.utils.unregister_class(PoseMapOperator)
    bpy.utils.unregister_class(SphereTestOperator)
    bpy.utils.unregister_class(TweakOperator)
    bpy.utils.unregister_class(RotateYOperator)
