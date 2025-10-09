using Unity.Mathematics;
using static UnityEngine.HumanBodyBones;

namespace HVR.IK.FullTiger
{
    public class HIKAvatarDefinition
    {
        /// When the avatar is in "artist pose", this is the length between the hip to the base of the neck.
        /// The IK solver must try to avoid straightening the curvature of the spinal chain.
        /// 
        // Avatar pose -- Relative to the parent bone. Model coordinates.
        internal readonly float3[] artistPosePos = new float3[(int)LastBone];
        internal readonly quaternion[] artistPoseRot = new quaternion[(int)LastBone];
        internal float refPoseHipToNeckLength;
        internal float refPoseHipToHeadLength;
        
        // Reference pose -- Relative to the parent bone. Model coordinates.
        internal readonly float3[] refPoseRelativePos = new float3[(int)LastBone];
        internal readonly quaternion[] refPoseRelativeRot = new quaternion[(int)LastBone];
        
        // Reference pose -- Relative to the hip bone. Model coordinates.
        internal readonly float3[] refPoseHiplativePos = new float3[(int)LastBone];
        internal readonly quaternion[] refPoseHiplativeRot = new quaternion[(int)LastBone];
        
        // Humanoid data -- Humanoid coordinates.
        internal readonly bool[] dataHasBone = new bool[(int)LastBone];
        internal readonly quaternion[] dataPostRot = new quaternion[(int)LastBone];
        internal readonly quaternion[] dataInversePostRot = new quaternion[(int)LastBone];
        
        internal readonly float4x4[] relativeMatrices = new float4x4[(int)LastBone];
        
        internal bool isInitialized;
    }
}