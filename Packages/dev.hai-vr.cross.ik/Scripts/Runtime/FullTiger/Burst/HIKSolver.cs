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
using System.Collections.Generic;
using Unity.Collections;
#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
using Unity.Mathematics;
using UnityEngine.Profiling;
#else //__iff HVR_IS_GODOT
using float3 = Godot.Vector3;
using float2 = Godot.Vector2;
using float4x4 = Godot.Transform3D;
using quaternion = Godot.Quaternion;
using math = hvr_godot_math;
#endif

namespace HVR.IK.FullTiger
{
    /// Given a definition and an objective, solves a pose into a snapshot.
    /// There is no dependency on the transform system beyond this point.
    /// Use Unity.Mathematics wherever applicable.
    internal class/*was_struct*/ HIKSolver
    {
        private readonly HIKSpineSolver _spineSolver;
        private readonly HIKArmSolver _armSolver;
        private readonly HIKLegSolver _legSolver;
        
        public HIKSolver(HIKAvatarDefinition definition, HIKLookupTables lookupTables = default)
        {
            if (!definition.isInitialized) throw new InvalidOperationException("definition must be initialized before instantiating the solver");
            
            // if (lookupTables == null) lookupTables = new HIKLookupTables();

            var reorienter = MbusGeofunctions.FromToOrientation(math.forward(), math.right(), math.up(), -math.up());
            _spineSolver = new HIKSpineSolver(definition, reorienter);
            _armSolver = new HIKArmSolver(definition, reorienter, lookupTables);
            _legSolver = new HIKLegSolver(definition, reorienter);
        }

        public HIKSnapshot Solve(HIKObjective objective, HIKSnapshot ikSnapshot, HIKLookupTables lookupTables, bool debugDrawSolver = false, HIKDebugDrawFlags debugDrawFlags = HIKDebugDrawFlags.Default)
        {
            if (objective.solveSpine)
            {
                helper_profiler_BeginSample("HIK Solve Spine");
                ikSnapshot = _spineSolver.Solve(objective, ikSnapshot, debugDrawSolver, debugDrawFlags);
                helper_profiler_EndSample();
            }
            
            // We need to solve the legs before the arms to support virtually parenting the hand effector to a bone of the leg.
            helper_profiler_BeginSample("HIK Solve Both Legs");
            ikSnapshot = _legSolver.Solve(objective, ikSnapshot, debugDrawSolver, debugDrawFlags);
            helper_profiler_EndSample();
            helper_profiler_BeginSample("HIK Rewrite Objectives");
            RewriteObjectiveToAccountForHandSelfParenting(ikSnapshot, objective.selfParentRightHandNullable, ref objective.rightHandTargetWorldPosition, ref objective.rightHandTargetWorldRotation);
            RewriteObjectiveToAccountForHandSelfParenting(ikSnapshot, objective.selfParentLeftHandNullable, ref objective.leftHandTargetWorldPosition, ref objective.leftHandTargetWorldRotation);
            helper_profiler_EndSample();
            helper_profiler_BeginSample("HIK Solve Both Arms");
            ikSnapshot = _armSolver.Solve(objective, ikSnapshot, lookupTables, debugDrawSolver, debugDrawFlags);
            helper_profiler_EndSample();
            return ikSnapshot;
        }

#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
        private void helper_profiler_BeginSample(string name) { Profiler.BeginSample(name); }
        private void helper_profiler_EndSample() { Profiler.EndSample(); }
#else //__iff HVR_IS_GODOT
        private void helper_profiler_BeginSample(string _) { }
        private void helper_profiler_EndSample() { }
#endif

        private void RewriteObjectiveToAccountForHandSelfParenting(HIKSnapshot ikSnapshot, HIKSelfParenting objectiveSelfParentHandNullable, ref float3 pos, ref quaternion rot)
        {
            if (objectiveSelfParentHandNullable is { } parent && parent.use > 0f)
            {
                var parentBone = (int)parent.bone;
                var trs = math.mul(
                    hvr_godot_helper.float4x4_TRUniform(ikSnapshot.absolutePos[parentBone], ikSnapshot.absoluteRot[parentBone]),
                    hvr_godot_helper.float4x4_TRUniform(parent.relPosition, parent.relRotation)
                );
                pos = math.lerp(pos, hvr_godot_helper.PositionOf(trs), parent.use);
                rot = math.slerp(rot, hvr_godot_helper.RotationOf(trs), parent.use);
            }
        }
    }

    internal struct/*converted_to_struct*/ HIKLookupTables : IDisposable
    {
        public bool isAvailable;
        
        private readonly HIKBendLookup _armBendLookup;

        public HIKLookupTables(List<float3> armBend)
        {
            _armBendLookup = new HIKBendLookup();
            _armBendLookup.init();
            _armBendLookup.ImportLookupTable(armBend);

            isAvailable = true;
        }

        public HIKLookupTables(NativeArray<float3> armBendLookupTable)
        {
            _armBendLookup = new HIKBendLookup();
            _armBendLookup.direct_init(armBendLookupTable);

            isAvailable = true;
        }

        public HIKBendLookup ArmBend()
        {
            return _armBendLookup;
        }

        public bool IsValid()
        {
            return isAvailable && _armBendLookup.IsValid();
        }

        public void Dispose()
        {
            _armBendLookup.Dispose();
        }
    }

    internal struct/*reconverted_to_struct*/ HIKObjective
    {
        public const float StruggleStart = 0.99f;
        public const float StruggleEnd = 1.04f;
        
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
        
        internal float useChest;
        internal float3 chestTargetWorldPosition;
        internal quaternion chestTargetWorldRotation;
        internal float alsoUseChestToMoveNeck;
        
        internal float useLeftLowerArm;
        internal float3 leftLowerArmWorldPosition;
        internal quaternion leftLowerArmWorldRotation; // TODO: We may have to play with the rotation to solve the "straight arm twist" problem.
        internal float useRightLowerArm;
        internal float3 rightLowerArmWorldPosition;
        internal quaternion rightLowerArmWorldRotation; // TODO: We may have to play with the rotation to solve the "straight arm twist" problem.

        internal float useLeftElbowPlaneCollider; // TODO: Not implemented
        internal float3 leftElbowPlaneWorldPosition; // TODO: Not implemented
        internal float3 leftElbowPlaneWorldNormal; // TODO: Not implemented
        
        internal bool headAlignmentMattersMore;
        internal bool allowContortionist;
        internal bool doNotPreserveHipsToNeckCurvatureLimit;
        
        internal float legStruggleStart;
        internal float legStruggleEnd;
        internal float armStruggleStart;
        internal float armStruggleEnd;
        
        internal float useShoulder;
        internal float shoulderForwardAngleMultiplier;
        internal float shoulderUpwardAngleMultiplier;
        
        internal float3 providedLossyScale;
        
        internal bool useStraddlingLeftLeg;
        internal bool useStraddlingRightLeg;
        internal float3 groundedStraddlingLeftLegWorldPosition;
        internal quaternion groundedStraddlingLeftLegWorldRotation;
        internal float3 groundedStraddlingRightLegWorldPosition;
        internal quaternion groundedStraddlingRightLegWorldRotation;
        
        internal bool solveSpine;
        internal bool solveLeftLeg;
        internal bool solveRightLeg;
        internal bool solveLeftArm;
        internal bool solveRightArm;
        
        internal float improveSpineBuckling;
        
        internal int fabrikIterations;
        internal bool useLookupTables;

        internal float __useFakeDoubleJointedKnees;

        // FIXME: Switching to structs for burst makes these no longer nullable
        internal HIKSelfParenting selfParentLeftHandNullable;
        internal HIKSelfParenting selfParentRightHandNullable;
    }

    internal struct/*reconverted_to_struct*/ HIKSelfParenting
    {
        public float use;
        public HIKBodyBones bone;
        public float3 relPosition;
        public quaternion relRotation;
    }

    [Flags]
    public enum HIKDebugDrawFlags
    {
        Default = 0,
        ShowSpine = 1,
        ShowArm = 2,
        ShowLeg = 4,
    }
}