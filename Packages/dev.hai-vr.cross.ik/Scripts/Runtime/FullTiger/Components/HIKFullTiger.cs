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
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
using UnityEngine.Profiling;
using static UnityEngine.HumanBodyBones;

namespace HVR.IK.FullTiger
{
    public class HIKFullTiger : MonoBehaviour
    {
        // private static readonly HumanBodyBones[] ShoulderAndArms = { LeftShoulder, LeftUpperArm, LeftLowerArm, LeftHand, RightShoulder, RightUpperArm, RightLowerArm, RightHand };
        // private static readonly HumanBodyBones[] HipsToHead = { Hips, Spine, Chest, UpperChest, Neck, Head };
        // private static readonly HumanBodyBones[] Legs = { LeftUpperLeg, LeftLowerLeg, LeftFoot, RightUpperLeg, RightLowerLeg, RightFoot };
        private static readonly HumanBodyBones[] AcceptableBodyBones =
        {
            Hips, Spine, Chest, UpperChest, Neck, Head,
            LeftShoulder, LeftUpperArm, LeftLowerArm, LeftHand,
            RightShoulder, RightUpperArm, RightLowerArm, RightHand,
            LeftUpperLeg, LeftLowerLeg, LeftFoot, LeftToes,
            RightUpperLeg, RightLowerLeg, RightFoot, RightToes
        };
        private static readonly HumanBodyBones[] CopyOrder =
        {
            /*Hips,*/ Spine, Chest, UpperChest, Neck, Head,
            LeftShoulder, LeftUpperArm, LeftLowerArm, LeftHand,
            RightShoulder, RightUpperArm, RightLowerArm, RightHand,
            LeftUpperLeg, LeftLowerLeg, LeftFoot, //LeftToes,
            RightUpperLeg, RightLowerLeg, RightFoot, //RightToes,
        };
        private static readonly HumanBodyBones[] CopyOrderSpineAndShoulders =
        {
            /*Hips,*/ Spine, Chest, UpperChest, Neck, Head,
            LeftShoulder,
            RightShoulder
        };
        private static readonly HumanBodyBones[] CopyOrderLeftArm =
        {
            LeftShoulder, LeftUpperArm, LeftLowerArm, LeftHand,
        };
        private static readonly HumanBodyBones[] CopyOrderRightArm =
        {
            RightShoulder, RightUpperArm, RightLowerArm, RightHand,
        };
        private static readonly HumanBodyBones[] CopyOrderLeftLeg =
        {
            LeftUpperLeg, LeftLowerLeg, LeftFoot, //LeftToes,
        };
        private static readonly HumanBodyBones[] CopyOrderRightLeg =
        {
            RightUpperLeg, RightLowerLeg, RightFoot, //RightToes,
        };
        
        private static readonly HumanBodyBones[] SpineChain = { Hips, Spine, Chest, UpperChest, Neck, Head };
        private static readonly HumanBodyBones[] LeftArmChain = { LeftShoulder, LeftUpperArm, LeftLowerArm, LeftHand };
        private static readonly HumanBodyBones[] RightArmChain = { RightShoulder, RightUpperArm, RightLowerArm, RightHand };
        
        [SerializeField] internal Animator animator;
        [SerializeField] internal HIKEffectors effectors;
        
        private HIKAvatarDefinition definition = new();
        private HIKSolver _ikSolver;
        
        private readonly Transform[] _bones = new Transform[(int)LastBone];

        [Header("Solver Settings")]
        public bool updateInLateUpdate = false;
        public bool useJobSystem = false;
        public bool updateEveryFrame = true;
        private HIKSnapshot _ikSnapshot = new();

        public bool overrideDefaultFabrikIterationCount = false;
        public int fabrikIterations = HIKSpineSolver.Iterations;
        public bool useLookupTables = true;
        
        [Header("Debug")]
        public bool debugDrawFinalChains = true;
        public bool debugDrawSolver = true;
        public HIKDebugDrawFlags debugDrawFlags = (HIKDebugDrawFlags)int.MaxValue;
        
        private bool _solveSpine = true;
        private bool _solveLeftLeg = true;
        private bool _solveRightLeg = true;
        private bool _solveLeftArm = true;
        private bool _solveRightArm = true;
        private bool _useFakeDoubleJointedKneesWasEverEnabled;
        
        private JobHandle _jobHandle;
        private NativeArray<HIKSnapshot> _result;

        private void Awake()
        {
            definition = SolveDefinition(animator, definition, _bones);
            
            // Order matters: This must be instantiated AFTER definition is initialized
            _ikSolver = new HIKSolver(definition, new HIKLookupTables(ParseLookup()));
        }

        private List<float3> ParseLookup()
        {
            // FIXME: The asset may not be available in a built app because it's not referenced
            var lookupTable = AssetDatabase.LoadAssetByGUID<TextAsset>(new GUID("dad70e4f1a7437a43b2cd4b25a877c67")); // This guid is arm_bend_lookup_table.txt
            var vectors = lookupTable.text.Split(';')
                .Select(s =>
                {
                    var v = s.Split(',');
                    return new float3(float.Parse(v[0]), float.Parse(v[1]), float.Parse(v[2]));
                })
                .ToList();
            return vectors;
        }

        internal static HIKAvatarDefinition SolveDefinition(Animator animator, HIKAvatarDefinition definition, Transform[] bones)
        {
            // TODO: We should T-Pose the avatar before sampling the hiplative positions
            var hips = animator.GetBoneTransform(Hips);
            foreach (var boneId in AcceptableBodyBones)
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
            
            definition.refPoseHipsLength = math.distance(definition.refPoseHiplativePos[(int)Hips], definition.refPoseHiplativePos[(int)Spine]);
            definition.refPoseSpineLength = math.distance(definition.refPoseHiplativePos[(int)Spine], definition.refPoseHiplativePos[(int)Chest]);
            definition.refPoseChestLength = math.distance(definition.refPoseHiplativePos[(int)Chest], definition.refPoseHiplativePos[(int)Neck]);
            definition.refPoseNeckLength = math.distance(definition.refPoseHiplativePos[(int)Neck], definition.refPoseHiplativePos[(int)Head]);

            {
                // Encode the spine curvature
                
                var spine = animator.GetBoneTransform(Spine);
                var chest = animator.GetBoneTransform(Chest);
                var neck = animator.GetBoneTransform(Neck);
                var head = animator.GetBoneTransform(Head);
                
                var hipsRef = math.mul(hips.rotation, definition.dataPostRot[(int)Hips]);
                var headRef = math.mul(head.rotation, definition.dataPostRot[(int)Head]);
                
                var spineToHead = (float3)(head.position - spine.position);
                var spineToHeadLength = math.length(spineToHead);
                var spineToHeadNormalized = math.normalize(spineToHead);

                var spineToChest = (float3)(chest.position - spine.position);
                var spineToNeck = (float3)(neck.position - spine.position);

                var hipsSide = math.mul(hipsRef, math.forward());
                var crossNormalized = math.normalize(math.cross(spineToHeadNormalized, hipsSide));

                definition.refPoseChestRelation = new float2(math.dot(spineToHeadNormalized, spineToChest) / spineToHeadLength, math.dot(crossNormalized, spineToChest));
                definition.refPoseNeckRelation = new float2(math.dot(spineToHeadNormalized, spineToNeck) / spineToHeadLength, math.dot(crossNormalized, spineToNeck));

                definition.refPoseSpineVecForHipsRotation = new float2(math.dot(math.mul(hipsRef, math.right()), spineToHeadNormalized), math.dot(math.mul(hipsRef, math.down()), spineToHeadNormalized));
                definition.refPoseSpineVecForHeadRotation = new float2(math.dot(math.mul(headRef, math.right()), spineToHeadNormalized), math.dot(math.mul(headRef, math.down()), spineToHeadNormalized));
            }
            
            definition.capturedWithLossyScale = bones[(int)Hips].lossyScale;
            definition.isInitialized = true;

            return definition;
        }

        private void Update()
        {
            if (useJobSystem)
            {
                PerformRegularSolveInJobSystem();
            }
            else
            {
                if (!updateInLateUpdate)
                {
                    ExecuteSolver();
                }
            }
        }

        private void LateUpdate()
        {
            if (useJobSystem)
            {
                _jobHandle.Complete();
                _ikSnapshot = _result[0];
                _result.Dispose();
                ApplySnapshot();
            }
            else
            {
                if (updateInLateUpdate)
                {
                    ExecuteSolver();
                }
            }
        }

        internal struct HIKFullTigerJob : IJob
        {
            public NativeArray<HIKSnapshot> result;
            public HIKObjective objective;
            public HIKAvatarDefinition definition;

            public void Execute()
            {
                var solver = new HIKSolver(definition);
                var snapshot = new HIKSnapshot();
                snapshot = solver.Solve(objective, snapshot);
                result[0] = snapshot;
            }
        }

        private void ExecuteSolver() 
        {
            if (!effectors.isActiveAndEnabled) return;
            if (!effectors.IsInitialized()) return;
            if (!updateEveryFrame) return;

            Profiler.BeginSample("HIK Perform Regular Solve");
            PerformRegularSolve();
            Profiler.EndSample();
            
            Profiler.BeginSample("HIK Apply Snapshot");
            ApplySnapshot();
            Profiler.EndSample();

#if UNITY_EDITOR && true
            if (debugDrawFinalChains)
            {
                DrawArmChain(SpineChain);
                DrawArmChain(LeftArmChain);
                DrawArmChain(RightArmChain);
            }
#endif
        }

        public void PerformRegularSolve()
        {
            var objective = CreateObjective();
            _ikSnapshot = _ikSolver.Solve(objective, _ikSnapshot, debugDrawSolver, debugDrawFlags);
        }

        private void PerformRegularSolveInJobSystem()
        {
            var objective = CreateObjective();
            
            _result = new NativeArray<HIKSnapshot>(1, Allocator.TempJob);
            var job = new HIKFullTigerJob
            {
                result = _result,
                objective = objective,
                definition = definition
            };
            _jobHandle = job.Schedule();
        }

        private HIKObjective CreateObjective()
        {
            Profiler.BeginSample("HIK Build HIKObjective");
            HIKSelfParenting selfParentLeftHand;
            if (effectors.useSelfParentLeftHand > 0f)
            {
                selfParentLeftHand = new HIKSelfParenting
                {
                    use = effectors.useSelfParentLeftHand,
                    bone = (HIKBodyBones)(int)effectors.selfParentLeftHandBone,
                    relPosition = effectors.selfParentLeftHandRelativePosition,
                    relRotation = quaternion.Euler(effectors.selfParentLeftHandRelativeRotationEuler),
                };
            }
            else
            {
                selfParentLeftHand = default;
            }
            HIKSelfParenting selfParentRightHand;
            if (effectors.useSelfParentRightHand > 0f)
            {
                selfParentRightHand = new HIKSelfParenting
                {
                    use = effectors.useSelfParentRightHand,
                    bone = (HIKBodyBones)(int)effectors.selfParentRightHandBone,
                    relPosition = effectors.selfParentRightHandRelativePosition,
                    relRotation = quaternion.Euler(effectors.selfParentRightHandRelativeRotationEuler),
                };
            }
            else
            {
                selfParentRightHand = default;
            }

            var objective = new HIKObjective
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
                
                useLeftLowerArm = effectors.useLeftLowerArm,
                leftLowerArmWorldPosition = effectors.leftLowerArmTarget.position,
                leftLowerArmWorldRotation = effectors.leftLowerArmTarget.rotation,
                useRightLowerArm = effectors.useRightLowerArm,
                rightLowerArmWorldPosition = effectors.rightLowerArmTarget.position,
                rightLowerArmWorldRotation = effectors.rightLowerArmTarget.rotation,
                
                headAlignmentMattersMore = !effectors.hipPositionMattersMore,
                allowContortionist = effectors.contortionist,
                doNotPreserveHipsToNeckCurvatureLimit = effectors.doNotPreserveHipsToNeckCurvatureLimit,
                
                useStraddlingLeftLeg = effectors.useStraddlingLeftLeg,
                useStraddlingRightLeg = effectors.useStraddlingRightLeg,
                groundedStraddlingLeftLegWorldPosition = effectors.groundedStraddlingLeftLeg.position,
                groundedStraddlingLeftLegWorldRotation = effectors.groundedStraddlingLeftLeg.rotation,
                groundedStraddlingRightLegWorldPosition = effectors.groundedStraddlingRightLeg.position,
                groundedStraddlingRightLegWorldRotation = effectors.groundedStraddlingRightLeg.rotation,
                
                solveSpine = _solveSpine,
                solveLeftLeg = _solveLeftLeg,
                solveRightLeg = _solveRightLeg,
                solveLeftArm = _solveLeftArm,
                solveRightArm = _solveRightArm,
                
                legStruggleStart = effectors.legStruggleStart,
                legStruggleEnd = effectors.legStruggleEnd,
                armStruggleStart = effectors.armStruggleStart,
                armStruggleEnd = effectors.armStruggleEnd,
                
                useShoulder = effectors.useShoulder,
                shoulderForwardAngleMultiplier = effectors.shoulderForwardAngleMultiplier,
                shoulderUpwardAngleMultiplier = effectors.shoulderUpwardAngleMultiplier,
                
                improveSpineBuckling = effectors.improveSpineBuckling,
                
                providedLossyScale = _bones[(int)Hips].lossyScale,
                
                fabrikIterations = overrideDefaultFabrikIterationCount ? fabrikIterations : HIKSpineSolver.Iterations,
                useLookupTables = useLookupTables,
                
                __useFakeDoubleJointedKnees = effectors.useFakeDoubleJointedKnees,
                
                selfParentLeftHandNullable = selfParentLeftHand,
                selfParentRightHandNullable = selfParentRightHand,
            };
            Profiler.EndSample();
            
            return objective;
        }

        public void ApplySnapshot()
        {
            _bones[(int)Hips].position = _ikSnapshot.absolutePos[(int)Hips];
            _bones[(int)Hips].rotation = ConvertSnapshotRotationToBoneRotation(_ikSnapshot, definition, Hips);
            if (_solveSpine) Apply(CopyOrderSpineAndShoulders);
            if (!_useFakeDoubleJointedKneesWasEverEnabled && effectors.useFakeDoubleJointedKnees <= 0f)
            {
                if (_solveLeftLeg) Apply(CopyOrderLeftLeg);
                if (_solveRightLeg) Apply(CopyOrderRightLeg);
            }
            else
            {
                _useFakeDoubleJointedKneesWasEverEnabled = true;
                if (_solveLeftLeg)
                {
                    Apply(LeftUpperLeg);
                    _bones[(int)LeftLowerLeg].position = _ikSnapshot.absolutePos[(int)LeftLowerLeg];
                    Apply(LeftLowerLeg, LeftFoot);
                    _bones[(int)LeftFoot].position = _ikSnapshot.absolutePos[(int)LeftFoot];
                };
                if (_solveRightLeg)
                {
                    Apply(RightUpperLeg);
                    _bones[(int)RightLowerLeg].position = _ikSnapshot.absolutePos[(int)RightLowerLeg];
                    Apply(RightLowerLeg, RightFoot);
                    _bones[(int)RightFoot].position = _ikSnapshot.absolutePos[(int)RightFoot];
                }
            }
            if (_solveLeftArm) Apply(CopyOrderLeftArm);
            if (_solveRightArm) Apply(CopyOrderRightArm);
        }

        private void Apply(params HumanBodyBones[] bones)
        {
            foreach (var boneId in bones)
            {
                var index = (int)boneId;
                if (definition.dataHasBone[index])
                {
                    _bones[index].rotation = ConvertSnapshotRotationToBoneRotation(_ikSnapshot, definition, boneId);
                }
            }
        }

        private static quaternion ConvertSnapshotRotationToBoneRotation(HIKSnapshot ikSnapshot, HIKAvatarDefinition definition, HumanBodyBones bone)
        {
            return math.mul(ikSnapshot.absoluteRot[(int)bone], definition.dataInversePostRot[(int)bone]);
        }

        private void DrawArmChain(HumanBodyBones[] array)
        {
#if UNITY_EDITOR && true
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
#endif
        }
    }
}
#endif