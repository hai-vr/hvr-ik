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
            LeftShoulder, // LeftUpperArm, LeftLowerArm, LeftHand,
            RightShoulder, RightUpperArm, RightLowerArm, RightHand,
            // LeftUpperLeg, LeftLowerLeg, LeftFoot, LeftToes,
            // RightUpperLeg, RightLowerLeg, RightFoot, RightToes,
        };
        private static readonly HumanBodyBones[] SpineChain = { Spine, Chest, UpperChest, Neck, Head };
        
        [SerializeField] internal Animator animator;
        [SerializeField] internal HIKEffectors effectors;
        
        private readonly HIKAvatarDefinition definition = new();
        private readonly HIKSnapshot ikSnapshot = new();
        private HIKSolver _ikSolver;
        
        private readonly Transform[] _bones = new Transform[(int)LastBone];
        // private readonly Transform[] _standins = new Transform[(int)LastBone];

        public bool updateEveryFrame = true;

        private void Awake()
        {
            var hips = animator.GetBoneTransform(Hips);
            for (var boneId = Hips; boneId < LastBone; boneId++)
            {
                var i = (int)boneId;
                var boneNullable = animator.GetBoneTransform(boneId);
                _bones[i] = boneNullable;
                if (boneNullable != null)
                {
                    var bone = boneNullable;
                    var postRot = MbusAnimatorUtil.ReflectiveGetPostRotation(animator.avatar, boneId);
                    // var standin = new GameObject($"Standin{Enum.GetName(typeof(HumanBodyBones), boneId)}")
                    // {
                    //     transform =
                    //     {
                    //         parent = transform,
                    //         position = bone.position,
                    //         rotation = bone.rotation * postRot
                    //     }
                    // };
                    // standin.SetActive(false);
                    // _standins[i] = standin.transform;
                    
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
            
            // ORDER MATTERS: This must be instantiated AFTER definition is initialized
            _ikSolver = new HIKSolver(definition, ikSnapshot);
        }

        private void Update()
        {
            if (!effectors.isActiveAndEnabled) return;
            if (!effectors.IsInitialized()) return;
            if (!updateEveryFrame) return;

            // We want all bones from "Hips" = 0 to "RightToes" = 20, plus "UpperChest" = 54.

            _ikSolver.Solve(new HIKObjective
            {
                hipTargetWorldPosition = effectors.hipTarget.position,
                hipTargetWorldRotation = effectors.hipTarget.rotation,
                
                headTargetWorldPosition = effectors.headTarget.position,
                headTargetWorldRotation = effectors.headTarget.rotation,
                
                rightHandTargetWorldPosition = effectors.rightHandTarget.position,
                rightHandTargetWorldRotation = effectors.rightHandTarget.rotation,
                
                headAlignmentMattersMore = true,
                allowContortionist = false
            });

            _bones[(int)Hips].position = ikSnapshot.absolutePos[(int)Hips];
            _bones[(int)Hips].rotation = math.mul(ikSnapshot.absoluteRot[(int)Hips], definition.dataInversePostRot[(int)Hips]);
            var prevPos = ikSnapshot.absolutePos[(int)Hips];
            foreach (var boneId in CopyOrder)
            {
                var index = (int)boneId;
                if (definition.dataHasBone[index])
                {
                    _bones[index].rotation = math.mul(ikSnapshot.absoluteRot[index], definition.dataInversePostRot[index]);

                    // var newPos = ikSnapshot.absolutePos[index];
                    // Debug.DrawLine(prevPos, newPos, boneId == Spine ? Color.red : Color.cyan, 0f, false);
                    // prevPos = newPos;

                    // FIXME: This is just for debugging
                    // _bones[index].position = ikSnapshot.absolutePos[index];
                }
            }
            prevPos = _bones[(int)Hips].position;
            foreach (var boneId in SpineChain)
            {
                var index = (int)boneId;
                if (definition.dataHasBone[index])
                {
                    var newPos = _bones[index].position;
                    Debug.DrawLine(prevPos, newPos, Color.green, 0f, false);
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
        
        internal float3 rightHandTargetWorldPosition;
        internal quaternion rightHandTargetWorldRotation;
        
        internal bool headAlignmentMattersMore;
        internal bool allowContortionist;
    }

    internal class HIKSolver
    {
        private readonly HIKAvatarDefinition definition;
        private readonly HIKSnapshot ikSnapshot;

        private readonly HIKSpineSolver _spineSolver;
        private readonly HIKArmSolver _armSolver;

        public HIKSolver(HIKAvatarDefinition definition, HIKSnapshot ikSnapshot)
        {
            if (!definition.isInitialized) throw new InvalidOperationException("definition must be initialized before instantiating the solver");
            
            this.definition = definition;
            this.ikSnapshot = ikSnapshot;

            var reorienter = MbusGeofunctions.FromToOrientation(Vector3.forward, Vector3.right, Vector3.up, -Vector3.up);
            _spineSolver = new HIKSpineSolver(definition, ikSnapshot, reorienter, this);
            _armSolver = new HIKArmSolver(definition, ikSnapshot, reorienter, this);
        }

        public void Solve(HIKObjective objective)
        {
            _spineSolver.Solve(objective);
            _armSolver.Solve(objective);
        }

        internal void SetSnapshotToReferencePose(HumanBodyBones boneId)
        {
            var index = (int)boneId;
            if (definition.dataHasBone[index])
            {
                ikSnapshot.absoluteRot[index] = definition.refPoseHiplativeRot[index];
            }
        }
    }
}