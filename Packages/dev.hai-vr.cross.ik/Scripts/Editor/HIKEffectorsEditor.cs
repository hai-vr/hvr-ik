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

using System.Collections.Generic;
using HVR.IK.FullTiger;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

namespace HVR.IK.Editor
{
    [CustomEditor(typeof(HIKEffectors))]
    public class HIKEffectorsEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            
            var my = (HIKEffectors)target;
            
            if (GUILayout.Button("Set chest to current position"))
            {
                var bone = HumanBodyBones.Chest;
                var boneTransform = my.animator.GetBoneTransform(bone);
                var parentPosition = boneTransform.position;
                var parentRotation = boneTransform.rotation * MbusAnimatorUtil.ReflectiveGetPostRotation(my.animator.avatar, bone);
                
                my.chestTarget.position = parentPosition;
                my.chestTarget.rotation = parentRotation;
            }
            if (GUILayout.Button("Make Left Hand relative"))
            {
                BakeIt(my, my.leftHandTarget, my.selfParentLeftHandBone, ref my.selfParentLeftHandRelativePosition, ref my.selfParentLeftHandRelativeRotationEuler, ref my.useSelfParentLeftHand);
            }
            if (GUILayout.Button("Make Right Hand relative"))
            {
                BakeIt(my, my.rightHandTarget, my.selfParentRightHandBone, ref my.selfParentRightHandRelativePosition, ref my.selfParentRightHandRelativeRotationEuler, ref my.useSelfParentRightHand);
            }
            if (GUILayout.Button("Copy leg struggle response graph to clipboard"))
            {
                var struggleCurve = HIKTwoBoneAlgorithms.CreateStruggleCurve();
                var a = 0.5f;
                var b = 1 - a;

                var startd = 1 - my.legStruggleStart;
                var graphStart = 1 - startd * 4;
                var graphEnd = my.legStruggleEnd + 0.005f;
                var lines = new List<string>();
                lines.Add("Input distance (%);Output distance (%);Without (deg);With (deg)");
                for (var inputDistance = graphStart; inputDistance <= graphEnd; inputDistance += 0.00025f)
                {
                    float finalDistance;
                    if (inputDistance >= my.legStruggleStart)
                    {
                        var lerpAmount = math.clamp(math.unlerp(my.legStruggleStart, my.legStruggleEnd, inputDistance), 0f, 1f);
                        var calculatedLength = math.lerp(my.legStruggleStart, 1f, struggleCurve.Evaluate(lerpAmount));
                        finalDistance = math.clamp(calculatedLength, 0f, 1f);
                    }
                    else
                    {
                        finalDistance = math.clamp(inputDistance, 0f, 1f);
                    }
                    var clampedDistance = math.clamp(inputDistance, 0f, 1f);

                    var without = math.degrees(math.acos((a * a + b * b - clampedDistance * clampedDistance) / (2 * a * b)));
                    var with = math.degrees(math.acos((a * a + b * b - finalDistance * finalDistance) / (2 * a * b)));
                    lines.Add($"{inputDistance};{finalDistance};{without};{with}");
                }
                EditorGUIUtility.systemCopyBuffer = string.Join(System.Environment.NewLine, lines);
            }
        }

        private static void BakeIt(HIKEffectors my, Transform target, HumanBodyBones bone, ref float3 pos, ref float3 rotEuler, ref float use)
        {
            var boneTransform = my.animator.GetBoneTransform(bone);
            var parentPosition = boneTransform.position;
            var parentRotation = boneTransform.rotation * MbusAnimatorUtil.ReflectiveGetPostRotation(my.animator.avatar, bone);
             
            // FIXME: Something about the rotation is incorrect
            var our = math.mul(
                math.inverse(float4x4.TRS(parentPosition, parentRotation, new float3(1, 1, 1))),
                float4x4.TRS(target.position, target.rotation, new float3(1, 1, 1))
            );

            pos = our.c3.xyz;
            rotEuler = ((Quaternion)new quaternion(our)).eulerAngles;
            use = 1f;
        }
    }
}