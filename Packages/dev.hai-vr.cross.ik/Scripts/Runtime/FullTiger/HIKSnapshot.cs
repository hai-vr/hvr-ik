using Unity.Mathematics;
using UnityEngine;

namespace HVR.IK.FullTiger
{
    internal class HIKSnapshot
    {
        internal float3[] absolutePos = new float3[(int)HumanBodyBones.LastBone];
        internal quaternion[] absoluteRot = new quaternion[(int)HumanBodyBones.LastBone];

        // Recalculates the absolute position of a bone, based on the position and rotation of its parent and its relative matrix.
        public void ReevaluatePosition(HumanBodyBones ourBone, HIKAvatarDefinition definition)
        {
            var ourIndex = (int)ourBone;
            var parentBone = ParentOf(ourBone, definition.dataHasBone[(int)HumanBodyBones.UpperChest]);
            
            var resolvedMatrix = math.mul(TRSOf(parentBone, definition), definition.relativeMatrices[ourIndex]);
            
            absolutePos[ourIndex] = resolvedMatrix.c3.xyz;
        }

        private float4x4 TRSOf(HumanBodyBones parentBone, HIKAvatarDefinition definition)
        {
            var parentIndex = (int)parentBone;
            return float4x4.TRS(absolutePos[parentIndex], math.mul(absoluteRot[parentIndex], definition.dataInversePostRot[parentIndex]), new float3(1, 1, 1));
        }

        private HumanBodyBones ParentOf(HumanBodyBones bone, bool hasUpperChest)
        {
            return bone switch
            {
                HumanBodyBones.Spine => HumanBodyBones.Hips,
                HumanBodyBones.Chest => HumanBodyBones.Spine,
                HumanBodyBones.UpperChest => HumanBodyBones.Chest,
                HumanBodyBones.Neck => hasUpperChest ? HumanBodyBones.UpperChest : HumanBodyBones.Chest,
                HumanBodyBones.Head => HumanBodyBones.Neck,
                HumanBodyBones.LeftShoulder => hasUpperChest ? HumanBodyBones.UpperChest : HumanBodyBones.Chest,
                HumanBodyBones.RightShoulder => hasUpperChest ? HumanBodyBones.UpperChest : HumanBodyBones.Chest,
                HumanBodyBones.LeftUpperArm => HumanBodyBones.LeftShoulder,
                HumanBodyBones.RightUpperArm => HumanBodyBones.RightShoulder,
                HumanBodyBones.LeftLowerArm => HumanBodyBones.LeftUpperArm,
                HumanBodyBones.RightLowerArm => HumanBodyBones.RightUpperArm,
                HumanBodyBones.LeftHand => HumanBodyBones.LeftLowerArm,
                HumanBodyBones.RightHand => HumanBodyBones.RightLowerArm,
                HumanBodyBones.LeftUpperLeg => HumanBodyBones.Hips,
                HumanBodyBones.RightUpperLeg => HumanBodyBones.Hips,
                HumanBodyBones.LeftLowerLeg => HumanBodyBones.LeftUpperLeg,
                HumanBodyBones.RightLowerLeg => HumanBodyBones.RightUpperLeg,
                HumanBodyBones.LeftFoot => HumanBodyBones.LeftLowerLeg,
                HumanBodyBones.RightFoot => HumanBodyBones.RightLowerLeg,
                HumanBodyBones.LeftToes => HumanBodyBones.LeftFoot,
                HumanBodyBones.RightToes => HumanBodyBones.RightFoot,
                _ => HumanBodyBones.Hips
            };
        }

        public void ApplyReferenceRotation(HumanBodyBones ourBone, HIKAvatarDefinition definition)
        {
            var ourIndex = (int)ourBone;
            
            var parentBone = ParentOf(ourBone, definition.dataHasBone[(int)HumanBodyBones.UpperChest]);
            var parentIndex = (int)parentBone;

            absoluteRot[ourIndex] = math.mul(
                math.mul(
                    // Take the parent rotation in model coordinates
                    math.mul(absoluteRot[parentIndex], definition.dataInversePostRot[parentIndex]),
                    // Apply the rotative rotation to those model coordinates
                    definition.refPoseRelativeRot[ourIndex]
                ),
                // Return to IK coordinates
                definition.dataPostRot[ourIndex]
            );
        }
    }
}