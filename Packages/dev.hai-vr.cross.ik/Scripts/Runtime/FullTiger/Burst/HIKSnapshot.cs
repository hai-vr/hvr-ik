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

using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace HVR.IK.FullTiger
{
    internal struct HIKSnapshot : IDisposable
    {
        public void init()
        {
            absolutePos = new NativeArray<float3>((int)HIKBodyBones.LastBone, Allocator.Temp);
            absoluteRot = new NativeArray<quaternion>((int)HIKBodyBones.LastBone, Allocator.Temp);
        }
        public void initPersistent()
        {
            absolutePos = new NativeArray<float3>((int)HIKBodyBones.LastBone, Allocator.Persistent);
            absoluteRot = new NativeArray<quaternion>((int)HIKBodyBones.LastBone, Allocator.Persistent);
        }
        
        internal NativeArray<float3> absolutePos;
        internal NativeArray<quaternion> absoluteRot;

        public void Dispose()
        {
            if (absolutePos.IsCreated) absolutePos.Dispose();
            if (absoluteRot.IsCreated) absoluteRot.Dispose();
        }

        // Recalculates the absolute position of a bone, based on the position and rotation of its parent and its relative matrix.
        public void ReevaluatePosition(HIKBodyBones ourBone, HIKAvatarDefinition definition, float scale)
        {
            var ourIndex = (int)ourBone;
            var parentBone = ParentOf(ourBone, definition.dataHasBone[(int)HIKBodyBones.UpperChest]);

            // FIXME: We're creating a lot of matrices here for repeated calls.
            var scaleMatrix = float4x4.Scale(scale);
            var resolvedMatrix = math.mul(TRSOf(parentBone, definition), math.mul(scaleMatrix, definition.relativeMatrices[ourIndex]));
            
            absolutePos[ourIndex] = resolvedMatrix.c3.xyz;
        }

        private float4x4 TRSOf(HIKBodyBones parentBone, HIKAvatarDefinition definition)
        {
            var parentIndex = (int)parentBone;
            return float4x4.TRS(absolutePos[parentIndex], math.mul(absoluteRot[parentIndex], definition.dataInversePostRot[parentIndex]), new float3(1, 1, 1));
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
            var ourIndex = (int)ourBone;
            
            var parentBone = ParentOf(ourBone, definition.dataHasBone[(int)HIKBodyBones.UpperChest]);
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