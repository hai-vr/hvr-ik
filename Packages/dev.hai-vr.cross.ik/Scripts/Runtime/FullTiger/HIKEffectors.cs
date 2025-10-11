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
using UnityEngine;

namespace HVR.IK.FullTiger
{
    /// Creates the end effectors that will be used by the IK solver.
    /// The end effectors placements should be modified by external modules before the IK solver runs.
    public class HIKEffectors : MonoBehaviour
    {
        [SerializeField] private Animator animator;

        /*[AutoOutput]*/ public Transform runtimeTargets;
        /*[AutoOutput]*/ public Transform hipTarget;
        /*[AutoOutput]*/ public Transform headTarget;
        /*[AutoOutput]*/ public Transform leftHandTarget;
        /*[AutoOutput]*/ public Transform rightHandTarget;
        /*[AutoOutput]*/ public Transform leftFootTarget;
        /*[AutoOutput]*/ public Transform rightFootTarget;
        /*[AutoOutput]*/ public Transform groundedStraddlingLeftLeg;
        /*[AutoOutput]*/ public Transform groundedStraddlingRightLeg;

        [Range(0, 1)]
        public float useChest;
        public Transform chestTarget;
        [Range(0, 1)]
        public float alsoUseChestToMoveNeck;

        public Vector3 metaHipToHeadVector;
        public float metaFeetSeparation;
        public float metaFeetHeightFromGround;
        
        public bool hipPositionMattersMore;
        public bool contortionist;
        
        public bool useStraddlingLeftLeg;
        public bool useStraddlingRightLeg;

        private Vector3[] _tPosePos;
        private Quaternion[] _tPoseRot;

        public bool IsInitialized() => _isInitialized;
        private bool _isInitialized;

        private void OnEnable()
        {
            runtimeTargets = MbusUtil.NewTransform("RuntimeTargets", transform);
            hipTarget = CreateTarget(HumanBodyBones.Hips, "HipTarget");
            headTarget = CreateTarget(HumanBodyBones.Head, "HeadTarget");
            leftHandTarget = CreateTarget(HumanBodyBones.LeftHand, "LeftHandTarget");
            rightHandTarget = CreateTarget(HumanBodyBones.RightHand, "RightHandTarget");
            leftFootTarget = CreateTarget(HumanBodyBones.LeftFoot, "LeftFootTarget");
            rightFootTarget = CreateTarget(HumanBodyBones.RightFoot, "RightFootTarget");
            groundedStraddlingLeftLeg = CreateTarget(HumanBodyBones.LeftLowerLeg, "GroundedStraddlingLeftLeg");
            groundedStraddlingRightLeg = CreateTarget(HumanBodyBones.RightLowerLeg, "GroundedStraddlingRightLeg");
            
            chestTarget = CreateTarget(HumanBodyBones.Chest, "ChestTarget");

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
