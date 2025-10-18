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

using Unity.Mathematics;

namespace HVR.IK.FullTiger
{
    public class/*was_struct*/ HIKAvatarDefinition
    {
        /// When the avatar is in "artist pose", this is the length between the hip to the base of the neck.
        /// The IK solver must try to avoid straightening the curvature of the spinal chain.
        /// 
        // Avatar pose -- Relative to the parent bone. Model coordinates.
        internal HIKBoneData<float3> artistPosePos = new();
        internal HIKBoneData<quaternion> artistPoseRot = new();
        internal float refPoseHipToNeckLength;
        internal float refPoseHipToHeadLength;
        internal float refPoseHipsLength;
        internal float refPoseSpineLength;
        internal float refPoseChestLength;
        internal float refPoseNeckLength;
        
        internal float2 refPoseChestRelation;
        internal float2 refPoseNeckRelation;
        internal float2 refPoseSpineVecForHipsRotation;
        internal float2 refPoseSpineVecForHeadRotation;
        
        // Reference pose -- Relative to the parent bone. Model coordinates.
        internal HIKBoneData<float3> refPoseRelativePos = new();
        internal HIKBoneData<quaternion> refPoseRelativeRot = new();
        
        // Reference pose -- Relative to the hip bone. Model coordinates.
        internal HIKBoneData<float3> refPoseHiplativePos = new();
        internal HIKBoneData<quaternion> refPoseHiplativeRot = new();
        
        // Humanoid data -- Humanoid coordinates.
        internal HIKBoneData<bool> dataHasBone = new();
        internal HIKBoneData<quaternion> dataPostRot = new();
        internal HIKBoneData<quaternion> dataInversePostRot = new();
        
        internal HIKBoneData<float4x4> relativeMatrices = new();

        internal float3 capturedWithLossyScale;
        internal bool isInitialized;
    }
}