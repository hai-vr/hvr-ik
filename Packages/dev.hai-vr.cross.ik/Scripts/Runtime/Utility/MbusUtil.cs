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
    internal static class MbusUtil
    {
#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
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

        public static void DrawArrow(float3 start, float3 end, Color color, float duration, bool depthTest, float3 planeNormal, float size = 0.01f)
        {
#if UNITY_EDITOR && true
            var arrowLength = math.distance(end, start);
            var actualSize = arrowLength / 2 > size ? size : arrowLength / 2;
            
            var dir = math.normalize(end - start);
            var cross = math.normalize(math.cross(dir, planeNormal));
            Debug.DrawLine(start, end, color, duration, depthTest);
            
            var baseLocation = end - dir * actualSize;
            Debug.DrawLine(end, baseLocation + cross * actualSize, color, duration, depthTest);
            Debug.DrawLine(end, baseLocation - cross * actualSize, color, duration, depthTest);
#endif
        }

        public static void DrawDirectionArrow(float3 start, float3 end, Color color, float duration, bool depthTest, float3 planeNormal, float size = 0.01f)
        {
#if UNITY_EDITOR && true
            var arrowLength = math.distance(end, start);
            var actualSize = arrowLength / 2 > size ? size : arrowLength / 2;
            
            var dir = math.normalize(end - start);
            var cross = math.normalize(math.cross(dir, planeNormal));
            Debug.DrawLine(start, end, color, duration, depthTest);

            var shift = dir * actualSize * 0.5f;
            var mid = (start + end) * 0.5f + shift;
            var baseLocation = mid - dir * actualSize;
            Debug.DrawLine(mid, baseLocation + cross * actualSize, color, duration, depthTest);
            Debug.DrawLine(mid, baseLocation - cross * actualSize, color, duration, depthTest);
            Debug.DrawLine(mid - shift * 2, baseLocation + cross * actualSize - shift * 2, color, duration, depthTest);
            Debug.DrawLine(mid - shift * 2, baseLocation - cross * actualSize - shift * 2, color, duration, depthTest);
#endif
        }
#endif
    }
}