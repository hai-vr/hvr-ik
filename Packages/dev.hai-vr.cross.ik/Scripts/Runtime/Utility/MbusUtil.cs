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