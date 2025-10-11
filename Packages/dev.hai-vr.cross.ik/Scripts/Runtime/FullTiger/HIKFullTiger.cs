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

using System;
using Unity.Mathematics;
using UnityEngine;
using static UnityEngine.HumanBodyBones;

namespace HVR.IK.FullTiger
{
    public class HIKFullTiger : MonoBehaviour
    {
        // private static readonly HumanBodyBones[] ShoulderAndArms = { LeftShoulder, LeftUpperArm, LeftLowerArm, LeftHand, RightShoulder, RightUpperArm, RightLowerArm, RightHand };
        // private static readonly HumanBodyBones[] HipsToHead = { Hips, Spine, Chest, UpperChest, Neck, Head };
        // private static readonly HumanBodyBones[] Legs = { LeftUpperLeg, LeftLowerLeg, LeftFoot, RightUpperLeg, RightLowerLeg, RightFoot };
        private static readonly HumanBodyBones[] CopyOrder =
        {
            /*Hips,*/ Spine, Chest, UpperChest, Neck, Head,
            LeftShoulder, LeftUpperArm, LeftLowerArm, LeftHand,
            RightShoulder, RightUpperArm, RightLowerArm, RightHand,
            LeftUpperLeg, LeftLowerLeg, LeftFoot, //LeftToes,
            RightUpperLeg, RightLowerLeg, RightFoot, //RightToes,
        };
        private static readonly HumanBodyBones[] SpineChain = { Hips, Spine, Chest, UpperChest, Neck, Head };
        private static readonly HumanBodyBones[] LeftArmChain = { LeftShoulder, LeftUpperArm, LeftLowerArm, LeftHand };
        private static readonly HumanBodyBones[] RightArmChain = { RightShoulder, RightUpperArm, RightLowerArm, RightHand };
        
        [SerializeField] internal Animator animator;
        [SerializeField] internal HIKEffectors effectors;
        
        private readonly HIKAvatarDefinition definition = new();
        private readonly HIKSnapshot ikSnapshot = new();
        private HIKSolver _ikSolver;
        
        private readonly Transform[] _bones = new Transform[(int)LastBone];

        public bool updateEveryFrame = true;

        private void Awake()
        {
            SolveDefinition(animator, definition, _bones);
            
            // Order matters: This must be instantiated AFTER definition is initialized
            _ikSolver = new HIKSolver(definition, ikSnapshot);
        }

        private static void SolveDefinition(Animator animator, HIKAvatarDefinition definition, Transform[] bones)
        {
            // TODO: We should T-Pose the avatar before sampling the hiplative positions
            var hips = animator.GetBoneTransform(Hips);
            for (var boneId = Hips; boneId < LastBone; boneId++)
            {
                var i = (int)boneId;
                var boneNullable = animator.GetBoneTransform(boneId);
                bones[i] = boneNullable;
                if (boneNullable != null)
                {
                    var bone = boneNullable;
                    var postRot = MbusAnimatorUtil.ReflectiveGetPostRotation(animator.avatar, boneId);
                    
                    definition.artistPosePos[i] = bone.localPosition;
                    definition.artistPoseRot[i] = bone.localRotation;
                    
                    definition.refPoseRelativePos[i] = bone.localPosition; // TODO
                    definition.refPoseRelativeRot[i] = bone.localRotation; // TODO
                    definition.refPoseHiplativePos[i] = hips.InverseTransformPoint(bone.position); // TODO
                    definition.refPoseHiplativeRot[i] = math.inverse(hips.rotation) * bone.rotation; // TODO
                    
                    definition.dataHasBone[i] = true;
                    definition.dataPostRot[i] = postRot;
                    definition.dataInversePostRot[i] = math.inverse(postRot);
                    
                    definition.relativeMatrices[i] = float4x4.TRS(bone.localPosition, bone.localRotation, bone.localScale);
                }
                else
                {
                    definition.artistPosePos[i] = float3.zero;
                    definition.artistPoseRot[i] = quaternion.identity;
                    
                    definition.refPoseRelativePos[i] = float3.zero;
                    definition.refPoseRelativeRot[i] = quaternion.identity;
                    definition.refPoseHiplativePos[i] = float3.zero;
                    definition.refPoseHiplativeRot[i] = quaternion.identity;
                    
                    definition.dataHasBone[i] = false;
                    definition.dataPostRot[i] = quaternion.identity;
                    definition.dataInversePostRot[i] = quaternion.identity;
                    
                    definition.relativeMatrices[i] = float4x4.identity;
                }
            }

            definition.refPoseHipToNeckLength = math.distance(definition.refPoseHiplativePos[(int)Hips], definition.refPoseHiplativePos[(int)Neck]);
            definition.refPoseHipToHeadLength = math.distance(definition.refPoseHiplativePos[(int)Hips], definition.refPoseHiplativePos[(int)Head]);
            definition.refPoseNeckLength = math.distance(definition.refPoseHiplativePos[(int)Neck], definition.refPoseHiplativePos[(int)Head]);
            definition.isInitialized = true;
        }

        private void Update()
        {
            if (!effectors.isActiveAndEnabled) return;
            if (!effectors.IsInitialized()) return;
            if (!updateEveryFrame) return;

            PerformRegularSolve();
            ApplySnapshot();

            DrawArmChain(SpineChain);
            DrawArmChain(LeftArmChain);
            DrawArmChain(RightArmChain);
        }

        public void PerformRegularSolve()
        {
            _ikSolver.Solve(new HIKObjective
            {
                hipTargetWorldPosition = effectors.hipTarget.position,
                hipTargetWorldRotation = effectors.hipTarget.rotation,
                
                headTargetWorldPosition = effectors.headTarget.position,
                headTargetWorldRotation = effectors.headTarget.rotation,
                
                leftHandTargetWorldPosition = effectors.leftHandTarget.position,
                leftHandTargetWorldRotation = effectors.leftHandTarget.rotation,
                rightHandTargetWorldPosition = effectors.rightHandTarget.position,
                rightHandTargetWorldRotation = effectors.rightHandTarget.rotation,
                
                leftFootTargetWorldPosition = effectors.leftFootTarget.position,
                leftFootTargetWorldRotation = effectors.leftFootTarget.rotation,
                rightFootTargetWorldPosition = effectors.rightFootTarget.position,
                rightFootTargetWorldRotation = effectors.rightFootTarget.rotation,
                
                useChest = effectors.useChest,
                chestTargetWorldPosition = effectors.chestTarget.position,
                chestTargetWorldRotation = effectors.chestTarget.rotation,
                alsoUseChestToMoveNeck = effectors.alsoUseChestToMoveNeck,
                
                headAlignmentMattersMore = !effectors.hipPositionMattersMore,
                allowContortionist = effectors.contortionist,
                
                useStraddlingLeftLeg = effectors.useStraddlingLeftLeg,
                useStraddlingRightLeg = effectors.useStraddlingRightLeg,
                groundedStraddlingLeftLegWorldPosition = effectors.groundedStraddlingLeftLeg.position,
                groundedStraddlingLeftLegWorldRotation = effectors.groundedStraddlingLeftLeg.rotation,
                groundedStraddlingRightLegWorldPosition = effectors.groundedStraddlingRightLeg.position,
                groundedStraddlingRightLegWorldRotation = effectors.groundedStraddlingRightLeg.rotation,
                
                solveSpine = true,
                solveLeftLeg = true,
                solveRightLeg = true,
                solveLeftArm = true,
                solveRightArm = true,
            });
        }

        public void ApplySnapshot()
        {
            _bones[(int)Hips].position = ikSnapshot.absolutePos[(int)Hips];
            _bones[(int)Hips].rotation = math.mul(ikSnapshot.absoluteRot[(int)Hips], definition.dataInversePostRot[(int)Hips]);
            foreach (var boneId in CopyOrder)
            {
                var index = (int)boneId;
                if (definition.dataHasBone[index])
                {
                    _bones[index].rotation = math.mul(ikSnapshot.absoluteRot[index], definition.dataInversePostRot[index]);
                }
            }
        }

        private void DrawArmChain(HumanBodyBones[] array)
        {
            float3 prevPos = _bones[(int)array[0]].position + Vector3.up * 0.001f;
            for (var i = 1; i < array.Length; i++)
            {
                var boneId = array[i];
                var index = (int)boneId;
                if (definition.dataHasBone[index])
                {
                    var newPos = _bones[index].position + Vector3.up * 0.001f;
                    Debug.DrawLine(prevPos, newPos, Color.blue, 0f, false);
                    prevPos = newPos;
                }
            }
        }
    }
    
    internal class HIKObjective
    {
        internal float3 hipTargetWorldPosition;
        internal quaternion hipTargetWorldRotation;
        internal float3 headTargetWorldPosition;
        internal quaternion headTargetWorldRotation;
        
        internal float3 leftHandTargetWorldPosition;
        internal quaternion leftHandTargetWorldRotation;
        internal float3 rightHandTargetWorldPosition;
        internal quaternion rightHandTargetWorldRotation;
        
        internal float3 leftFootTargetWorldPosition;
        internal quaternion leftFootTargetWorldRotation;
        internal float3 rightFootTargetWorldPosition;
        internal quaternion rightFootTargetWorldRotation;
        
        public float useChest;
        internal float3 chestTargetWorldPosition;
        internal quaternion chestTargetWorldRotation;
        internal float alsoUseChestToMoveNeck;
        
        internal bool headAlignmentMattersMore;
        internal bool allowContortionist;
        
        public bool useStraddlingLeftLeg;
        public bool useStraddlingRightLeg;
        internal float3 groundedStraddlingLeftLegWorldPosition;
        internal quaternion groundedStraddlingLeftLegWorldRotation;
        internal float3 groundedStraddlingRightLegWorldPosition;
        internal quaternion groundedStraddlingRightLegWorldRotation;
        
        public bool solveSpine;
        public bool solveLeftLeg;
        public bool solveRightLeg;
        public bool solveLeftArm;
        public bool solveRightArm;
    }

    /// Solves given a definition and an objective, solves a pose into a snapshot.
    /// There is no dependency on the transform system beyond this point.
    /// Use Unity.Mathematics wherever applicable.
    internal class HIKSolver
    {
        private readonly HIKSpineSolver _spineSolver;
        private readonly HIKArmSolver _armSolver;
        private readonly HIKLegSolver _legSolver;

        public HIKSolver(HIKAvatarDefinition definition, HIKSnapshot ikSnapshot)
        {
            if (!definition.isInitialized) throw new InvalidOperationException("definition must be initialized before instantiating the solver");

            var reorienter = MbusGeofunctions.FromToOrientation(Vector3.forward, Vector3.right, Vector3.up, -Vector3.up);
            _spineSolver = new HIKSpineSolver(definition, ikSnapshot, reorienter);
            _armSolver = new HIKArmSolver(definition, ikSnapshot, reorienter);
            _legSolver = new HIKLegSolver(definition, ikSnapshot, reorienter);
        }

        public void Solve(HIKObjective objective)
        {
            if (objective.solveSpine) _spineSolver.Solve(objective);
            _armSolver.Solve(objective);
            _legSolver.Solve(objective);
        }
    }
}