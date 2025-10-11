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

using System.Linq;
using Unity.Mathematics;
using UnityEngine;

namespace HVR.IK.FullTiger
{
    /// Creates the end effectors that will be used by the IK solver.
    /// The end effectors placements should be modified by external modules before the IK solver runs.
    public class HIKEffectors : MonoBehaviour
    {
        [SerializeField] private Animator animator;

        public Transform runtimeTargets;
        public Transform hipTarget;
        public Transform headTarget;
        public Transform leftHandTarget;
        public Transform rightHandTarget;
        public Transform leftFootTarget;
        public Transform rightFootTarget;
        public Transform groundedStraddlingLeftLeg;
        public Transform groundedStraddlingRightLeg;

        [Range(0, 1)]
        public float useChest;
        public Transform chestTarget;
        [Range(0, 1)]
        public float alsoUseChestToMoveNeck;
        
        public bool hipPositionMattersMore;
        public bool contortionist;
        
        public bool useStraddlingLeftLeg;
        public bool useStraddlingRightLeg;
        
        [Range(0, 1)]
        public float useSelfParentRightHand;
        public HumanBodyBones selfParentRightHandBone;
        public float3 selfParentRightHandRelativePosition;
        public float3 selfParentRightHandRelativeRotationEuler;
        
        [Range(0, 1)]
        public float useSelfParentLeftHand;
        public HumanBodyBones selfParentLeftHandBone;
        public float3 selfParentLeftHandRelativePosition;
        public float3 selfParentLeftHandRelativeRotationEuler;

        private Vector3[] _tPosePos;
        private Quaternion[] _tPoseRot;

        public bool IsInitialized() => _isInitialized;
        private bool _isInitialized;

        private void OnEnable()
        {
            if (null == runtimeTargets) runtimeTargets = MbusUtil.NewTransform("RuntimeTargets", transform);
            if (null == hipTarget) hipTarget = CreateTarget(HumanBodyBones.Hips, "HipTarget");
            if (null == headTarget) headTarget = CreateTarget(HumanBodyBones.Head, "HeadTarget");
            if (null == leftHandTarget) leftHandTarget = CreateTarget(HumanBodyBones.LeftHand, "LeftHandTarget");
            if (null == rightHandTarget) rightHandTarget = CreateTarget(HumanBodyBones.RightHand, "RightHandTarget");
            if (null == leftFootTarget) leftFootTarget = CreateTarget(HumanBodyBones.LeftFoot, "LeftFootTarget");
            if (null == rightFootTarget) rightFootTarget = CreateTarget(HumanBodyBones.RightFoot, "RightFootTarget");
            if (null == groundedStraddlingLeftLeg) groundedStraddlingLeftLeg = CreateTarget(HumanBodyBones.LeftLowerLeg, "GroundedStraddlingLeftLeg");
            if (null == groundedStraddlingRightLeg) groundedStraddlingRightLeg = CreateTarget(HumanBodyBones.RightLowerLeg, "GroundedStraddlingRightLeg");
            
            if (null == chestTarget) chestTarget = CreateTarget(HumanBodyBones.Chest, "ChestTarget");

            _tPosePos = AllTargetsStartingWithHead().Select(t => t.position).ToArray();
            _tPoseRot = AllTargetsStartingWithHead().Select(t => t.rotation).ToArray();
            
            _isInitialized = true;
        }

        private void OnDisable()
        {
            hipTarget = null;
            headTarget = null;
            leftHandTarget = null;
            rightHandTarget = null;
            leftFootTarget = null;
            rightFootTarget = null;
            groundedStraddlingLeftLeg = null;
            groundedStraddlingRightLeg = null;

            Destroy(runtimeTargets.gameObject);
            runtimeTargets = null;
        }

        public void ApplyTPoseAndRepositionHeadToMatch(Vector3 pos, Quaternion rot)
        {
            ApplyTPose();
            var transferRotation = rot * Quaternion.Inverse(headTarget.rotation);
            var others = AllTargetsStartingWithHead().Skip(1).ToArray();
            var repositions = others.Select(t => headTarget.InverseTransformPoint(t.position)).ToArray();
            headTarget.position = pos;
            headTarget.rotation = rot;
            for (var index = 0; index < others.Length; index++)
            {
                var t = others[index];
                t.position = headTarget.TransformPoint(repositions[index]);
                t.rotation = transferRotation * t.rotation;
            }
        }

        public void ApplyTPose()
        {
            var targets = AllTargetsStartingWithHead();
            for (var index = 0; index < targets.Length; index++)
            {
                var target = targets[index];
                target.position = _tPosePos[index];
                target.rotation = _tPoseRot[index];
            }
        }

        private Transform[] AllTargetsStartingWithHead()
        {
            return new [] { headTarget, hipTarget, leftHandTarget, rightHandTarget, leftFootTarget, rightFootTarget };
        }

        private Transform CreateTarget(HumanBodyBones which, string targetName)
        {
            return CreateTarget(MbusAnimatorUtil.ReflectiveGetPostRotation(animator.avatar, which), animator.GetBoneTransform(which), targetName);
        }

        private Transform CreateTarget(Quaternion postRotation, Transform bone, string targetName)
        {
            return MbusUtil.NewTransform(targetName, runtimeTargets, bone.position, bone.rotation * postRotation);
        }
    }
}
