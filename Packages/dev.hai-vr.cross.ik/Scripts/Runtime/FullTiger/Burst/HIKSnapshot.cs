// Copyright 2025 Haï~ (@vr_hai github.com/hai-vr)
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//    http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
using Unity.Mathematics;
#else //__iff HVR_IS_GODOT
using float3 = Godot.Vector3;
using float2 = Godot.Vector2;
using float4x4 = Godot.Transform3D;
using quaternion = Godot.Quaternion;
using math = hvr_godot_math;
#endif

namespace HVR.IK.FullTiger
{
    internal struct/*reconverted_to_struct*/ HIKSnapshot
    {
        internal HIKBoneData<float3> absolutePos;// = new();
        internal HIKBoneData<quaternion> absoluteRot;// = new();

        // Recalculates the absolute position of a bone, based on the position and rotation of its parent and its relative matrix.
        public void ReevaluatePosition(HIKBodyBones ourBone, HIKAvatarDefinition definition, float scale)
        {
            var ourIndex = (int)ourBone;
            var parentBone = ParentOf(ourBone, definition.dataHasBone[(int)HIKBodyBones.UpperChest]);

            // FIXME: We're creating a lot of matrices here for repeated calls.
            var scaleMatrix = hvr_godot_helper.float4x4_Scale(scale);
            var resolvedMatrix = math.mul(TRSOf(parentBone, definition), math.mul(scaleMatrix, definition.relativeMatrices[ourIndex]));
            
            absolutePos[ourIndex] = hvr_godot_helper.PositionOf(resolvedMatrix);
        }

        private float4x4 TRSOf(HIKBodyBones parentBone, HIKAvatarDefinition definition)
        {
            var parentIndex = (int)parentBone;
            return hvr_godot_helper.float4x4_TRUniform(absolutePos[parentIndex], math.mul(absoluteRot[parentIndex], definition.dataInversePostRot[parentIndex]));
        }

        private HIKBodyBones ParentOf(HIKBodyBones bone, bool hasUpperChest)
        {
            return bone switch
            {
                HIKBodyBones.Spine => HIKBodyBones.Hips,
                HIKBodyBones.Chest => HIKBodyBones.Spine,
                HIKBodyBones.UpperChest => HIKBodyBones.Chest,
                HIKBodyBones.Neck => hasUpperChest ? HIKBodyBones.UpperChest : HIKBodyBones.Chest,
                HIKBodyBones.Head => HIKBodyBones.Neck,
                HIKBodyBones.LeftShoulder => hasUpperChest ? HIKBodyBones.UpperChest : HIKBodyBones.Chest,
                HIKBodyBones.RightShoulder => hasUpperChest ? HIKBodyBones.UpperChest : HIKBodyBones.Chest,
                HIKBodyBones.LeftUpperArm => HIKBodyBones.LeftShoulder,
                HIKBodyBones.RightUpperArm => HIKBodyBones.RightShoulder,
                HIKBodyBones.LeftLowerArm => HIKBodyBones.LeftUpperArm,
                HIKBodyBones.RightLowerArm => HIKBodyBones.RightUpperArm,
                HIKBodyBones.LeftHand => HIKBodyBones.LeftLowerArm,
                HIKBodyBones.RightHand => HIKBodyBones.RightLowerArm,
                HIKBodyBones.LeftUpperLeg => HIKBodyBones.Hips,
                HIKBodyBones.RightUpperLeg => HIKBodyBones.Hips,
                HIKBodyBones.LeftLowerLeg => HIKBodyBones.LeftUpperLeg,
                HIKBodyBones.RightLowerLeg => HIKBodyBones.RightUpperLeg,
                HIKBodyBones.LeftFoot => HIKBodyBones.LeftLowerLeg,
                HIKBodyBones.RightFoot => HIKBodyBones.RightLowerLeg,
                HIKBodyBones.LeftToes => HIKBodyBones.LeftFoot,
                HIKBodyBones.RightToes => HIKBodyBones.RightFoot,
                _ => HIKBodyBones.Hips
            };
        }

        public void ApplyReferenceRotation(HIKBodyBones ourBone, HIKAvatarDefinition definition)
        {
            ApplyRelativeRotation(ourBone, definition, hvr_godot_helper.quaternion_identity);
        }

        public void ApplyRelativeRotation(HIKBodyBones ourBone, HIKAvatarDefinition definition, quaternion relativeRot)
        {
            var ourIndex = (int)ourBone;
            
            var parentBone = ParentOf(ourBone, definition.dataHasBone[(int)HIKBodyBones.UpperChest]);
            var parentIndex = (int)parentBone;

            absoluteRot[ourIndex] = math.mul(
                math.mul(
                    // Take the parent rotation in model coordinates
                    math.mul(math.mul(absoluteRot[parentIndex], relativeRot), definition.dataInversePostRot[parentIndex]),
                    // Apply the rotative rotation to those model coordinates
                    definition.refPoseRelativeRot[ourIndex]
                ),
                // Return to IK coordinates
                definition.dataPostRot[ourIndex]
            );
        }

        public void MakeTPoseWithoutRespectingArtistCurvature()
        {
            var body = quaternion.LookRotation(math.left(), math.back());
            absoluteRot.eltHips = body;
            absoluteRot.eltSpine = body;
            absoluteRot.eltChest = body;
            absoluteRot.eltUpperChest = body;
            absoluteRot.eltNeck = body;
            absoluteRot.eltHead = body;
            
            var legs = quaternion.LookRotation(math.right(), math.back());
            absoluteRot.eltLeftUpperLeg = legs;
            absoluteRot.eltLeftLowerLeg = legs;
            absoluteRot.eltLeftFoot = legs;
            absoluteRot.eltLeftToes = legs;
            absoluteRot.eltRightUpperLeg = legs;
            absoluteRot.eltRightLowerLeg = legs;
            absoluteRot.eltRightFoot = legs;
            absoluteRot.eltRightToes = legs;

            var leftSide = quaternion.LookRotation(math.forward(), math.down());
            // FIXME: Wait, something's weird about this. If we apply leftSide to the lower arm, this twists the arm, so this doesn't seem correct.
            // We're creating a new quaternion called "SideTwist" to address this, but there may be deeper issue here???
            var leftSideTwist = quaternion.LookRotation(math.up(), math.forward());
            absoluteRot.eltLeftShoulder = leftSide;
            absoluteRot.eltLeftUpperArm = leftSide;
            absoluteRot.eltLeftLowerArm = leftSideTwist; // ???
            absoluteRot.eltLeftHand = leftSide;
            
            var rightSide = quaternion.LookRotation(math.forward(), math.up()); // Same as identity in Unity
            var rightSideTwist = quaternion.LookRotation(math.up(), math.back());
            absoluteRot.eltRightShoulder = rightSide;
            absoluteRot.eltRightUpperArm = rightSide;
            absoluteRot.eltRightLowerArm = rightSideTwist; // ???
            absoluteRot.eltRightHand = rightSide;
        }
    }
}