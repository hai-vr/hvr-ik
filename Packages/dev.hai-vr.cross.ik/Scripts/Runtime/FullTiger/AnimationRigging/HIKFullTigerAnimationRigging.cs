#if HVR_IK_HAS_ANIMATION_RIGGING
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
using UnityEngine.Animations;
using UnityEngine.Animations.Rigging;
using static HVR.IK.FullTiger.HIKBodyBones;

namespace HVR.IK.FullTiger.AnimationRigging
{
    public class HIKFullTigerAnimationRigging : RigConstraint<HIKFullTigerAnimationRiggingJob, HIKFullTigerAnimationRiggingData, HIKFullTigerAnimationBinder<HIKFullTigerAnimationRiggingData>>
    {
            
    }

    [Unity.Burst.BurstCompile]
    public struct HIKFullTigerAnimationRiggingJob : IWeightedAnimationJob
    {
        private static readonly HIKBodyBones[] CopyOrderSpineAndShoulders =
        {
            /*Hips,*/ Spine, Chest, UpperChest, Neck, Head,
            LeftShoulder,
            RightShoulder
        };
        private static readonly HIKBodyBones[] CopyOrderLeftArm =
        {
            LeftShoulder, LeftUpperArm, LeftLowerArm, LeftHand,
        };
        private static readonly HIKBodyBones[] CopyOrderRightArm =
        {
            RightShoulder, RightUpperArm, RightLowerArm, RightHand,
        };
        private static readonly HIKBodyBones[] CopyOrderLeftLeg =
        {
            LeftUpperLeg, LeftLowerLeg, LeftFoot, //LeftToes,
        };
        private static readonly HIKBodyBones[] CopyOrderRightLeg =
        {
            RightUpperLeg, RightLowerLeg, RightFoot, //RightToes,
        };
        
        internal HIKAvatarDefinition definition;
        internal ReadWriteTransformHandle[] boundBones;
        internal HIKSolver solver;
        
        private float3 _pp;
        private quaternion _rr;

        public FloatProperty jobWeight { get; set; }
        
        public void ProcessAnimation(AnimationStream stream)
        {
            if (jobWeight.Get(stream) <= 0) return;

            HIKSnapshot ikSnapshot = default;
            ikSnapshot.init();
            var objective = new HIKObjective
            {
                hipTargetWorldPosition = math.up() + math.left() * 5,
                hipTargetWorldRotation = quaternion.identity,
                headTargetWorldPosition = math.up() * 2 + math.left() * 5,
                headTargetWorldRotation = quaternion.identity,
                
                leftHandTargetWorldPosition = math.left(),
                leftHandTargetWorldRotation = quaternion.identity,
                rightHandTargetWorldPosition = math.right(),
                rightHandTargetWorldRotation = quaternion.identity,
                
                leftFootTargetWorldPosition = float3.zero,
                leftFootTargetWorldRotation = quaternion.identity,
                rightFootTargetWorldPosition = float3.zero,
                rightFootTargetWorldRotation = quaternion.identity,
                
                useChest = 0f,
                chestTargetWorldPosition = float3.zero,
                chestTargetWorldRotation = quaternion.identity,
                alsoUseChestToMoveNeck = 0f,
                
                useLeftLowerArm = 0f,
                leftLowerArmWorldPosition = float3.zero,
                leftLowerArmWorldRotation = quaternion.identity,
                useRightLowerArm = 0f,
                rightLowerArmWorldPosition = float3.zero,
                rightLowerArmWorldRotation = quaternion.identity,

                useLeftElbowPlaneCollider = 0f,
                leftElbowPlaneWorldPosition = float3.zero,
                leftElbowPlaneWorldNormal = float3.zero,
                
                headAlignmentMattersMore = false,
                allowContortionist = false,
                doNotPreserveHipsToNeckCurvatureLimit = false,
                
                useStraddlingLeftLeg = false,
                useStraddlingRightLeg = false,
                groundedStraddlingLeftLegWorldPosition = float3.zero,
                groundedStraddlingLeftLegWorldRotation = quaternion.identity,
                groundedStraddlingRightLegWorldPosition = float3.zero,
                groundedStraddlingRightLegWorldRotation = quaternion.identity,
                
                solveSpine = true,
                solveLeftLeg = true,
                solveRightLeg = true,
                solveLeftArm = true,
                solveRightArm = true,
                
                legStruggleStart = HIKObjective.StruggleStart,
                legStruggleEnd = HIKObjective.StruggleEnd,
                armStruggleStart = HIKObjective.StruggleStart,
                armStruggleEnd = HIKObjective.StruggleEnd,
                
                fabrikIterations = HIKSpineSolver.Iterations,

                selfParentLeftHandNullable = new HIKSelfParenting
                {
                    use = 0f,
                    bone = HIKBodyBones.Hips,
                    relPosition = float3.zero,
                    relRotation = quaternion.identity
                },
                selfParentRightHandNullable = new HIKSelfParenting
                {
                    use = 0f,
                    bone = HIKBodyBones.Hips,
                    relPosition = float3.zero,
                    relRotation = quaternion.identity
                }
            };
            ikSnapshot = solver.Solve(objective, ikSnapshot);
            
            _pp = ikSnapshot.absolutePos[(int)Hips];
            _rr = ConvertSnapshotRotationToBoneRotation(ikSnapshot, definition, Hips);
            boundBones[(int)Hips].SetPosition(stream, _pp);
            boundBones[(int)Hips].SetRotation(stream, _rr);
            
            if (objective.solveSpine) Apply(stream, ikSnapshot, CopyOrderSpineAndShoulders);
            if (objective.solveLeftLeg) Apply(stream, ikSnapshot, CopyOrderLeftLeg);
            if (objective.solveRightLeg) Apply(stream, ikSnapshot, CopyOrderRightLeg);
            if (objective.solveLeftArm) Apply(stream, ikSnapshot, CopyOrderLeftArm);
            if (objective.solveRightArm) Apply(stream, ikSnapshot, CopyOrderRightArm);
            
            ikSnapshot.Dispose();
        }

        private void Apply(AnimationStream stream, HIKSnapshot ikSnapshot, HIKBodyBones[] array)
        {
            foreach (var boneId in array)
            {
                var index = (int)boneId;
                if (definition.dataHasBone[index])
                {
                    boundBones[index].SetRotation(stream, ConvertSnapshotRotationToBoneRotation(ikSnapshot, definition, boneId));
                }
            }
        }

        public void ProcessRootMotion(AnimationStream stream) { }

        private static quaternion ConvertSnapshotRotationToBoneRotation(HIKSnapshot ikSnapshot, HIKAvatarDefinition definition, HIKBodyBones bone)
        {
            return math.mul(ikSnapshot.absoluteRot[(int)bone], definition.dataInversePostRot[(int)bone]);
        }
    }

    public struct HIKFullTigerAnimationRiggingData : IAnimationJobData, IHIKFullTigerAnimationRiggingData
    {
        [SerializeField] public bool m_hipPositionMattersMore;
        [SerializeField] public bool m_contortionist;
        [SerializeField] public bool m_doNotPreserveHipsToNeckCurvatureLimit;

        public Transform[] bones { get; }
        public bool hipPositionMattersMore { get => m_hipPositionMattersMore; set => m_hipPositionMattersMore = value; }
        public bool contortionist { get => m_contortionist; set => m_contortionist = value; }
        public bool doNotPreserveHipsToNeckCurvatureLimit { get => m_doNotPreserveHipsToNeckCurvatureLimit; set => m_doNotPreserveHipsToNeckCurvatureLimit = value; }
        
        public bool IsValid()
        {
            Debug.Log("2");
            // TODO
            return true;
        }

        public void SetDefaultValues()
        {
        }
    }

    public interface IHIKFullTigerAnimationRiggingData
    {
        Transform[] bones { get; }
        
        bool hipPositionMattersMore { get; }
        bool contortionist { get; }
        bool doNotPreserveHipsToNeckCurvatureLimit { get; }
    }

    public class HIKFullTigerAnimationBinder<T> : AnimationJobBinder<HIKFullTigerAnimationRiggingJob, T> where T : struct, IAnimationJobData, IHIKFullTigerAnimationRiggingData
    {
        public override HIKFullTigerAnimationRiggingJob Create(Animator animator, ref T data, Component component)
        {
            var definition = new HIKAvatarDefinition();
            definition.init();

            var unboundBones = new Transform[(int)LastBone];
            definition = HIKFullTiger.SolveDefinition(animator, definition, unboundBones);
            var boundBones = unboundBones.Select(t => t != null ? ReadWriteTransformHandle.Bind(animator, t) : default).ToArray();
            
            var solver = new HIKSolver(definition);

            var job = new HIKFullTigerAnimationRiggingJob
            {
                definition = definition,
                boundBones = boundBones,
                solver = solver,
            };

            return job;
        }

        public override void Destroy(HIKFullTigerAnimationRiggingJob job) { }
    }
}
#endif