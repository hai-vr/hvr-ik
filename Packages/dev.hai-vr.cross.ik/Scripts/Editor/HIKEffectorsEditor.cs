using HVR.IK.FullTiger;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

namespace HVR.IK.Editor
{
    [CustomEditor(typeof(HIKEffectors))]
    public class HIKEffectorsEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            
            var my = (HIKEffectors)target;
            
            if (GUILayout.Button("Set chest to current position"))
            {
                var bone = HumanBodyBones.Chest;
                var boneTransform = my.animator.GetBoneTransform(bone);
                var parentPosition = boneTransform.position;
                var parentRotation = boneTransform.rotation * MbusAnimatorUtil.ReflectiveGetPostRotation(my.animator.avatar, bone);
                
                my.chestTarget.position = parentPosition;
                my.chestTarget.rotation = parentRotation;
            }
            if (GUILayout.Button("Make Left Hand relative"))
            {
                BakeIt(my, my.leftHandTarget, my.selfParentLeftHandBone, ref my.selfParentLeftHandRelativePosition, ref my.selfParentLeftHandRelativeRotationEuler, ref my.useSelfParentLeftHand);
            }
            if (GUILayout.Button("Make Right Hand relative"))
            {
                BakeIt(my, my.rightHandTarget, my.selfParentRightHandBone, ref my.selfParentRightHandRelativePosition, ref my.selfParentRightHandRelativeRotationEuler, ref my.useSelfParentRightHand);
            }
        }

        private static void BakeIt(HIKEffectors my, Transform target, HumanBodyBones bone, ref float3 pos, ref float3 rotEuler, ref float use)
        {
            var boneTransform = my.animator.GetBoneTransform(bone);
            var parentPosition = boneTransform.position;
            var parentRotation = boneTransform.rotation * MbusAnimatorUtil.ReflectiveGetPostRotation(my.animator.avatar, bone);
             
            // FIXME: Something about the rotation is incorrect
            var our = math.mul(
                math.inverse(float4x4.TRS(parentPosition, parentRotation, new float3(1, 1, 1))),
                float4x4.TRS(target.position, target.rotation, new float3(1, 1, 1))
            );

            pos = our.c3.xyz;
            rotEuler = ((Quaternion)new quaternion(our)).eulerAngles;
            use = 1f;
        }
    }
}