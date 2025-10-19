using System;
#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
using Unity.Mathematics;
using UnityEngine;
#else //__iff HVR_IS_GODOT
using Godot;
using float3 = Godot.Vector3;
using float2 = Godot.Vector2;
using float4x4 = Godot.Transform3D;
using quaternion = Godot.Quaternion;
using math = hvr_godot_math;
#endif

namespace HVR.IK.FullTiger
{
    public static class hvr_godot_helper
    {
        public static readonly float3 float3_zero =
#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
            float3.zero;
#else //__iff HVR_IS_GODOT
            float3.Zero;
#endif
    
        public static readonly quaternion quaternion_identity =
#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
            quaternion.identity;
#else //__iff HVR_IS_GODOT
            quaternion.Identity;
#endif
        
        public static float4x4 float4x4_Scale(float scale)
        {
#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
            return float4x4.Scale(scale);
#else
            var scaleBasis = Basis.FromScale(new Vector3(scale, scale, scale));
            return new Transform3D(scaleBasis, Vector3.Zero);
#endif
        }
        
        public static float4x4 float4x4_TRS(float3 translation, quaternion rotation, float3 scale)
        {
#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
            return float4x4.TRS(translation, rotation, scale);
#else
            var basis = new Basis(rotation).Scaled(scale);
            return new Transform3D(basis, translation);
#endif
        }
        
        public static float4x4 float4x4_TRUniform(float3 translation, quaternion rotation)
        {
#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
            return float4x4.TRS(translation, rotation, new float3(1, 1, 1));
#else
            var basis = new Basis(rotation);
            return new Transform3D(basis, translation);
#endif
        }
    }

    public static class hvr_godot_helper_quaternion
    {
        public static quaternion AxisAngle(float3 axis, float angle)
        {
#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
            return quaternion.AxisAngle(axis, angle);
#else //__iff HVR_IS_GODOT
            return new quaternion(axis, angle);
#endif
        }
    
        public static quaternion LookRotationSafe(float3 forward, float3 upward)
        {
#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
            return quaternion.LookRotationSafe(forward, upward);
#else //__iff HVR_IS_GODOT
            if (forward.LengthSquared() < 1e-6f) return quaternion.Identity;
                
            var forwardNormalized = forward.Normalized();
            var up = upward.LengthSquared() < 1e-6f ? float3.Up : upward.Normalized();
            
            var dot = math.abs(math.dot(forwardNormalized, up));
            if (dot > 0.99f) up = math.abs(forwardNormalized.X) < 0.9f ? float3.Right : float3.Forward;
            
            var right = math.cross(up, forwardNormalized).Normalized();
            up = math.cross(forwardNormalized, right).Normalized();
            
            var basis = new Godot.Basis(right, up, forwardNormalized);
            return new quaternion(basis);
#endif
        }
    }
}