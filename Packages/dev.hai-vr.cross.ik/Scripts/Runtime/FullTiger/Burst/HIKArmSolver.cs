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
using Unity.Burst;
using Unity.Mathematics;
using UnityEngine;

namespace HVR.IK.FullTiger
{
    internal struct HIKArmSolver
    {
        private const float InsideSwitchingMul = 2;
        private readonly HIKAvatarDefinition definition;
        private readonly quaternion _reorienter;
        
        public HIKArmSolver(HIKAvatarDefinition definition, quaternion reorienter)
        {
            if (!definition.isInitialized) throw new InvalidOperationException("definition must be initialized before instantiating the solver");
            
            this.definition = definition;
            _reorienter = reorienter;
        }

        public HIKSnapshot Solve(HIKObjective objective, HIKSnapshot ikSnapshot)
        {
            // TODO: Add the ability for the solver to derive a lower arm sub-effector based on (the lower arm effector????? and) a L/R plane effector,
            // describing the intersection of two planes (a line) where the elbow joint could rest on. If the hand is at a fixed position, then the solution is the intersection between
            // a circle and the plane. The circle is described by the bend point rotating around the axis defined by the root pos and the objective pos.
            if (objective.solveRightArm) ikSnapshot = SolveArm(ikSnapshot, objective, ArmSide.Right, objective.rightHandTargetWorldPosition, objective.rightHandTargetWorldRotation);
            if (objective.solveLeftArm) ikSnapshot = SolveArm(ikSnapshot, objective, ArmSide.Left, objective.leftHandTargetWorldPosition, objective.leftHandTargetWorldRotation);
            return ikSnapshot;
        }

        private HIKSnapshot SolveArm(HIKSnapshot ikSnapshot, HIKObjective objective, ArmSide side, float3 originalObjectivePos, quaternion originalObjectiveRot)
        {
            var rootBone = side == ArmSide.Right ? HIKBodyBones.RightUpperArm : HIKBodyBones.LeftUpperArm;
            var midBone = side == ArmSide.Right ? HIKBodyBones.RightLowerArm : HIKBodyBones.LeftLowerArm;
            var tipBone = side == ArmSide.Right ? HIKBodyBones.RightHand : HIKBodyBones.LeftHand;
            
            var rootPos = ikSnapshot.absolutePos[(int)rootBone];

            var upperLength = math.distance(definition.refPoseHiplativePos[(int)rootBone], definition.refPoseHiplativePos[(int)midBone]);
            var lowerLength = math.distance(definition.refPoseHiplativePos[(int)midBone], definition.refPoseHiplativePos[(int)tipBone]);

            // Corrections
            var TODO_STRADDLING_IS_FALSE = false;
            var TODO_NO_STRUGGLE = 1f;
            var objectivePos = HIKTwoBoneAlgorithms.ApplyCorrections(originalObjectivePos, TODO_STRADDLING_IS_FALSE, rootPos, upperLength, lowerLength, out var distanceType, TODO_NO_STRUGGLE, TODO_NO_STRUGGLE);
            
            // TODO: Handle HasUpperChest
            var chestReference = ikSnapshot.absoluteRot[(int)HIKBodyBones.Chest];
            var bendDirection = TODO_STRADDLING_IS_FALSE ? float3.zero : ArmBendHeuristics(); // Bend direction is not used when straddling.

            float3 ArmBendHeuristics()
            {
                var useBend = side == ArmSide.Right ? objective.useRightLowerArm : objective.useLeftLowerArm;
                var midPoint = (objectivePos + rootPos) * 0.5f;
                var directedBend = math.normalize((side == ArmSide.Right ? objective.rightLowerArmWorldPosition : objective.leftLowerArmWorldPosition) - midPoint);
                if (useBend >= 1f)
                {
                    return directedBend;
                }
                
                var chestUpwards = math.mul(chestReference, math.right());
                
                var outwards = math.mul(chestReference, side == ArmSide.Right ? math.back() : math.forward());
                var handSource = math.mul(originalObjectiveRot, math.left());
                var palmDirection = math.mul(originalObjectiveRot, side == ArmSide.Right ? math.down() : math.up());

                var isOutwards = math.dot(outwards, -handSource);
                var isPalmUp = math.dot(chestUpwards, palmDirection);
                var isInside = math.clamp(math.smoothstep(0f, 1f, math.dot(-outwards, math.normalize(objectivePos - rootPos) * InsideSwitchingMul)), -1f, 1f);

#if UNITY_EDITOR && true
                Debug.DrawLine(rootPos , rootPos + chestUpwards * isOutwards * 0.1f, Color.red, 0f, false);
                Debug.DrawLine(rootPos + outwards * 0.01f, rootPos + outwards * 0.01f + chestUpwards * isPalmUp * 0.1f, Color.green, 0f, false);
                Debug.DrawLine(rootPos + outwards * 0.02f, rootPos + outwards * 0.02f + chestUpwards * isInside * 0.1f, Color.blue, 0f, false);
#endif
                
                var chestSource = math.mul(chestReference, math.left());
                var chestSourceBendingOutwards = math.normalize(chestSource + outwards * math.clamp(isInside, 0f, 1f));
                var step2 = MbusGeofunctions.LerpDot(handSource, handSource, chestSourceBendingOutwards, isPalmUp);
                var step3 = MbusGeofunctions.LerpDot(step2, step2, chestSourceBendingOutwards, isOutwards);
                var regular = MbusGeofunctions.LerpDot(step3, step3, chestSourceBendingOutwards, isInside);
                
                return useBend <= 0 ? regular : math.lerp(regular, directedBend, useBend);
            }
            
            // Solve
            var TODO_NO_STRADDLING_POSITION = float3.zero;
            float3 bendPointPos;
            (objectivePos, bendPointPos) = HIKTwoBoneAlgorithms.SolveBendPoint(rootPos, objectivePos, originalObjectiveRot, upperLength, lowerLength, TODO_STRADDLING_IS_FALSE, TODO_NO_STRADDLING_POSITION, distanceType, bendDirection);

            var twistBase = math.mul(chestReference, math.left());
            ikSnapshot.absoluteRot[(int)rootBone] = math.mul(
                quaternion.LookRotationSafe(bendPointPos - rootPos, side == ArmSide.Right ? twistBase : -twistBase),
                _reorienter
            );
            ikSnapshot.absoluteRot[(int)midBone] = math.mul(
                quaternion.LookRotationSafe(objectivePos - bendPointPos, MbusGeofunctions.ReprojectTwistToArm(objectivePos - bendPointPos, math.mul(originalObjectiveRot, math.right()), math.mul(originalObjectiveRot, side == ArmSide.Right ? math.forward() : math.back()))),
                _reorienter
            );
            ikSnapshot.absoluteRot[(int)tipBone] = originalObjectiveRot;
            ikSnapshot.ReevaluatePosition(rootBone, definition);
            ikSnapshot.ReevaluatePosition(midBone, definition);
            ikSnapshot.ReevaluatePosition(tipBone, definition);

            return ikSnapshot;
        }

        private enum ArmSide
        {
            Left,
            Right,
        }
    }
}