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

using System.Reflection;
#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
using Unity.Mathematics;
using UnityEngine;
#else //__iff HVR_IS_GODOT
using float3 = Godot.Vector3;
using float2 = Godot.Vector2;
using float4x4 = Godot.Transform3D;
using quaternion = Godot.Quaternion;
using math = hvr_godot_math;
#endif

namespace HVR.IK.FullTiger
{
    /// Please only call this from the main thread.
    internal static class MbusAnimatorUtil
    {
#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
        private static readonly MethodInfo ReflectiveGetPostRotationFn = AvatarPrivateInstanceMethod("GetPostRotation");
        private static readonly MethodInfo ReflectiveGetPreRotationFn = AvatarPrivateInstanceMethod("GetPreRotation");
        private static readonly MethodInfo ReflectiveGetLimitSignFn = AvatarPrivateInstanceMethod("GetLimitSign");

        private static readonly object[] SharedArray = new object[] { null };

        public static quaternion ReflectiveGetPostRotation(Avatar avatar, HumanBodyBones bone)
        {
            SharedArray[0] = bone;
            return (Quaternion)ReflectiveGetPostRotationFn.Invoke(avatar, SharedArray);
        }

        public static quaternion ReflectiveGetPreRotation(Avatar avatar, HumanBodyBones bone)
        {
            SharedArray[0] = bone;
            return (Quaternion)ReflectiveGetPreRotationFn.Invoke(avatar, SharedArray);
        }

        public static float3 ReflectiveGetLimitSign(Avatar avatar, HumanBodyBones bone)
        {
            SharedArray[0] = bone;
            return (Vector3)ReflectiveGetLimitSignFn.Invoke(avatar, SharedArray);
        }

        private static MethodInfo AvatarPrivateInstanceMethod(string methodName) =>
            typeof(Avatar).GetMethod(methodName, BindingFlags.NonPublic | BindingFlags.Instance);
#endif
    }
}
