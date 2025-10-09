using System.Reflection;
using Unity.Mathematics;
using UnityEngine;

namespace HVR.IK.FullTiger
{
    /// Please only call this from the main thread.
    internal static class MbusAnimatorUtil
    {
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
    }
}
