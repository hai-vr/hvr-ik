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

using HVR.IK.FullTiger;
using UnityEditor;
using UnityEngine;

namespace HVR.IK.Editor
{
    [CustomEditor(typeof(HIKFullTiger))]
    public class HIKFullTigerEditor : UnityEditor.Editor
    {
        private static readonly HumanBodyBones[] HipsToHead = {
            HumanBodyBones.Hips,
            HumanBodyBones.Spine,
            HumanBodyBones.Chest,
            HumanBodyBones.UpperChest,
            HumanBodyBones.Neck,
            HumanBodyBones.Head };

        private void OnSceneGUI()
        {
            if (Application.isPlaying) return;
            
            var my = (HIKFullTiger)target;
            
            var animator = my.animator;
            if (animator != null)
            {
                var previous = Vector3.zero;
                foreach (var bone in HipsToHead)
                {
                    var boneTransform = animator.GetBoneTransform(bone);
                    if (boneTransform != null)
                    {
                        var pos = boneTransform.position;
                        if (bone != HumanBodyBones.Hips)
                        {
                            Debug.DrawLine(previous, pos, Color.red, 0f, false);
                        }
                        previous = pos;
                    }
                }
            }
        }

        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            if (GUILayout.Button("Create animation clip from pose"))
            {
                var clip = CreateAnimationClipFromPose(((HIKFullTiger)target).animator);
                Selection.activeObject = clip;
                EditorGUIUtility.PingObject(clip);
            }
        }

        private AnimationClip CreateAnimationClipFromPose(Animator animator)
        {
            var clip = new AnimationClip();
            
            var poseHandler = new HumanPoseHandler(animator.avatar, animator.transform);
            var humanPose = new HumanPose();
            poseHandler.GetHumanPose(ref humanPose);

            clip.SetCurve("", typeof(Animator), "RootT.x", AnimationCurve.Constant(0, 0, humanPose.bodyPosition.x));
            clip.SetCurve("", typeof(Animator), "RootT.y", AnimationCurve.Constant(0, 0, humanPose.bodyPosition.y));
            clip.SetCurve("", typeof(Animator), "RootT.z", AnimationCurve.Constant(0, 0, humanPose.bodyPosition.z));
            
            clip.SetCurve("", typeof(Animator), "RootQ.x", AnimationCurve.Constant(0, 0, humanPose.bodyRotation.x));
            clip.SetCurve("", typeof(Animator), "RootQ.y", AnimationCurve.Constant(0, 0, humanPose.bodyRotation.y));
            clip.SetCurve("", typeof(Animator), "RootQ.z", AnimationCurve.Constant(0, 0, humanPose.bodyRotation.z));
            clip.SetCurve("", typeof(Animator), "RootQ.w", AnimationCurve.Constant(0, 0, humanPose.bodyRotation.w));

            for (var i = 0; i < humanPose.muscles.Length && i < HumanTrait.MuscleCount; i++)
            {
                var muscleName = HumanTrait.MuscleName[i];
                var muscleValue = humanPose.muscles[i];

                var binding = EditorCurveBinding.FloatCurve("", typeof(Animator), muscleName);
                var curve = AnimationCurve.Constant(0, 0, muscleValue);
                
                AnimationUtility.SetEditorCurve(clip, binding, curve);
            }

            return clip;
        }
    }
}