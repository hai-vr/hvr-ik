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
    }
}