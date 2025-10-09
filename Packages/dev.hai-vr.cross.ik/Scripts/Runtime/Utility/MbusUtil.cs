using UnityEngine;

namespace HVR.IK.FullTiger
{
    internal static class MbusUtil
    {
        public static Transform NewTransform(string transformName, Transform parent)
        {
            return NewTransform(transformName, parent, parent.position, parent.rotation);
        }

        public static Transform NewTransform(string transformName, Transform parent, Vector3 pos, Quaternion rot)
        {
            return new GameObject(transformName)
            {
                transform =
                {
                    parent = parent,
                    position = pos,
                    rotation = rot
                }
            }.transform;
        }
        
        public static void CopyPosRot(Transform from, Transform to)
        {
            to.position = from.position;
            to.rotation = from.rotation;
        }

        public static void CopyPosRot(Transform from, Transform to, Quaternion postRotation)
        {
            to.position = from.position;
            to.rotation = from.rotation * postRotation;
        }
        
        public static void CopyPosRotLocalScale(Transform from, Transform to)
        {
            to.position = from.position;
            to.rotation = from.rotation;
            to.localScale = from.localScale;
        }

        public static void CopyPosRotLocalScale(Transform from, Transform to, Quaternion postRotation)
        {
            to.position = from.position;
            to.rotation = from.rotation * postRotation;
            to.localScale = from.localScale;
        }
    }
}