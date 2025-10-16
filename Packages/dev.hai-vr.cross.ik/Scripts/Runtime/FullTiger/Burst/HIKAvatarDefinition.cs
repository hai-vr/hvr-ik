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
using static HVR.IK.FullTiger.HIKBodyBones;

namespace HVR.IK.FullTiger
{
    public struct HIKAvatarDefinition : IDisposable
    {
        /// When the avatar is in "artist pose", this is the length between the hip to the base of the neck.
        /// The IK solver must try to avoid straightening the curvature of the spinal chain.
        /// 
        // Avatar pose -- Relative to the parent bone. Model coordinates.
        internal NativeArray<float3> artistPosePos;
        internal NativeArray<quaternion> artistPoseRot;
        internal float refPoseHipToNeckLength;
        internal float refPoseHipToHeadLength;
        internal float refPoseChestLength;
        internal float refPoseNeckLength;
        
        internal float2 refPoseChestRelation;
        internal float2 refPoseNeckRelation;
        internal float2 refPoseSpineVecForHipsRotation;
        internal float2 refPoseSpineVecForHeadRotation;
        
        // Reference pose -- Relative to the parent bone. Model coordinates.
        internal NativeArray<float3> refPoseRelativePos;
        internal NativeArray<quaternion> refPoseRelativeRot;
        
        // Reference pose -- Relative to the hip bone. Model coordinates.
        internal NativeArray<float3> refPoseHiplativePos;
        internal NativeArray<quaternion> refPoseHiplativeRot;
        
        // Humanoid data -- Humanoid coordinates.
        internal NativeArray<bool> dataHasBone;
        internal NativeArray<quaternion> dataPostRot;
        internal NativeArray<quaternion> dataInversePostRot;
        
        internal NativeArray<float4x4> relativeMatrices;

        internal float3 capturedWithLossyScale;
        internal bool isInitialized;

        public void init()
        {
            var boneCount = (int)LastBone;
            
            artistPosePos = new NativeArray<float3>(boneCount, Allocator.Persistent);
            artistPoseRot = new NativeArray<quaternion>(boneCount, Allocator.Persistent);
            refPoseRelativePos = new NativeArray<float3>(boneCount, Allocator.Persistent);
            refPoseRelativeRot = new NativeArray<quaternion>(boneCount, Allocator.Persistent);
            refPoseHiplativePos = new NativeArray<float3>(boneCount, Allocator.Persistent);
            refPoseHiplativeRot = new NativeArray<quaternion>(boneCount, Allocator.Persistent);
            dataHasBone = new NativeArray<bool>(boneCount, Allocator.Persistent);
            dataPostRot = new NativeArray<quaternion>(boneCount, Allocator.Persistent);
            dataInversePostRot = new NativeArray<quaternion>(boneCount, Allocator.Persistent);
            relativeMatrices = new NativeArray<float4x4>(boneCount, Allocator.Persistent);
            
            isInitialized = true;
        }

        public void Dispose()
        {
            if (artistPosePos.IsCreated) artistPosePos.Dispose();
            if (artistPoseRot.IsCreated) artistPoseRot.Dispose();
            if (refPoseRelativePos.IsCreated) refPoseRelativePos.Dispose();
            if (refPoseRelativeRot.IsCreated) refPoseRelativeRot.Dispose();
            if (refPoseHiplativePos.IsCreated) refPoseHiplativePos.Dispose();
            if (refPoseHiplativeRot.IsCreated) refPoseHiplativeRot.Dispose();
            if (dataHasBone.IsCreated) dataHasBone.Dispose();
            if (dataPostRot.IsCreated) dataPostRot.Dispose();
            if (dataInversePostRot.IsCreated) dataInversePostRot.Dispose();
            if (relativeMatrices.IsCreated) relativeMatrices.Dispose();
            
            isInitialized = false;
        }
    }
}