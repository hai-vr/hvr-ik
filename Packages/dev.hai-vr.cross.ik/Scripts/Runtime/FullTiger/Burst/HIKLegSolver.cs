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

namespace HVR.IK.FullTiger
{
    internal class/*was_struct*/ HIKLegSolver
    {
        private readonly HIKAvatarDefinition definition;
        private readonly quaternion _reorienter;

        public HIKLegSolver(HIKAvatarDefinition definition, quaternion reorienter)
        {
            if (!definition.isInitialized) throw new InvalidOperationException("definition must be initialized before instantiating the solver");
            
            this.definition = definition;
            _reorienter = reorienter;
        }

        public HIKSnapshot Solve(HIKObjective objective, HIKSnapshot ikSnapshot, bool debugDrawSolver)
        {
            var scale = math.length(objective.providedLossyScale) / math.length(definition.capturedWithLossyScale);

            if (objective.solveRightLeg) ikSnapshot = SolveLeg(ikSnapshot, objective, LegSide.Right, objective.rightFootTargetWorldPosition, objective.rightFootTargetWorldRotation, objective.useStraddlingRightLeg, objective.groundedStraddlingRightLegWorldPosition, objective.groundedStraddlingRightLegWorldRotation, scale, debugDrawSolver);
            if (objective.solveLeftLeg) ikSnapshot = SolveLeg(ikSnapshot, objective, LegSide.Left, objective.leftFootTargetWorldPosition, objective.leftFootTargetWorldRotation, objective.useStraddlingLeftLeg, objective.groundedStraddlingLeftLegWorldPosition, objective.groundedStraddlingLeftLegWorldRotation, scale, debugDrawSolver);
            return ikSnapshot;
        }

        private HIKSnapshot SolveLeg(HIKSnapshot ikSnapshot, HIKObjective objective, LegSide side, float3 originalObjectivePos, quaternion originalObjectiveRot, bool useStraddlingLeg, float3 groundedStraddlingLegWorldPosition, quaternion groundedStraddlingLegWorldRotation, float scale, bool debugDrawSolver)
        {
            var rootBone = side == LegSide.Right ? HIKBodyBones.RightUpperLeg : HIKBodyBones.LeftUpperLeg;
            var midBone = side == LegSide.Right ? HIKBodyBones.RightLowerLeg : HIKBodyBones.LeftLowerLeg;
            var tipBone = side == LegSide.Right ? HIKBodyBones.RightFoot : HIKBodyBones.LeftFoot;
            
            var rootPos = ikSnapshot.absolutePos[(int)rootBone];

            var upperLength = math.distance(definition.refPoseHiplativePos[(int)rootBone], definition.refPoseHiplativePos[(int)midBone]) * scale;
            var lowerLength = math.distance(definition.refPoseHiplativePos[(int)midBone], definition.refPoseHiplativePos[(int)tipBone]) * scale;
            
            // Corrections
            var objectivePos = HIKTwoBoneAlgorithms.ApplyCorrections(originalObjectivePos, useStraddlingLeg, rootPos, upperLength, lowerLength, out var distanceType, objective.legStruggleStart, objective.legStruggleEnd, debugDrawSolver);

            var hipReference = ikSnapshot.absoluteRot[(int)HIKBodyBones.Hips];
            
            var bendDirection = useStraddlingLeg ? float3.zero : LegBendHeuristics(); // Bend direction is not used when straddling.

            float3 LegBendHeuristics()
            {
                return math.normalize(math.mul(originalObjectiveRot, math.down() + math.left()));
            }

            // Solve
            float3 bendPointPos;
            (objectivePos, bendPointPos) = HIKTwoBoneAlgorithms.SolveBendPoint(rootPos, objectivePos, originalObjectiveRot, upperLength, lowerLength, useStraddlingLeg, groundedStraddlingLegWorldPosition, distanceType, bendDirection, debugDrawSolver);

            ikSnapshot.absoluteRot[(int)rootBone] = math.mul(
                quaternion.LookRotationSafe(bendPointPos - rootPos, math.normalize(math.cross(math.mul(hipReference, math.forward()), bendPointPos - rootPos))),
                _reorienter
            );
            ikSnapshot.absoluteRot[(int)midBone] = math.mul(
                quaternion.LookRotationSafe(objectivePos - bendPointPos, math.normalize(math.cross(math.mul(originalObjectiveRot, math.back()), objectivePos - bendPointPos))),
                _reorienter
            );
            ikSnapshot.absoluteRot[(int)tipBone] = originalObjectiveRot;
            ikSnapshot.ReevaluatePosition(rootBone, definition, scale);
            ikSnapshot.ReevaluatePosition(midBone, definition, scale);
            ikSnapshot.ReevaluatePosition(tipBone, definition, scale);

            if (objective.__useFakeDoubleJointedKnees > 0f)
            {
                var angleDeg = math.degrees(math.acos(math.clamp(math.dot(math.normalize(bendPointPos - rootPos), math.normalize(bendPointPos - objectivePos)), -1f, 1f)));
                var acuteness = 1 - math.clamp(math.unlerp(0f, 90f, angleDeg), 0f, 1f);
                
                var fakeMidPointPos = ikSnapshot.absolutePos[(int)midBone] + math.normalize(objectivePos - rootPos) * (lowerLength / 3.5f * acuteness * objective.__useFakeDoubleJointedKnees);
                ikSnapshot.absolutePos[(int)midBone] = fakeMidPointPos;
                ikSnapshot.absoluteRot[(int)midBone] = math.mul(
                    quaternion.LookRotationSafe(objectivePos - fakeMidPointPos, math.normalize(math.cross(math.mul(originalObjectiveRot, math.back()), objectivePos - fakeMidPointPos))),
                    _reorienter
                );
#if UNITY_EDITOR && true
                if (debugDrawSolver && acuteness > 0f)
                {
                    MbusUtil.DrawArrow(bendPointPos, fakeMidPointPos, Color.magenta, 0f, false, math.mul(ikSnapshot.absoluteRot[(int)midBone], math.forward()));
                    Debug.DrawLine(fakeMidPointPos, objectivePos, Color.cyan, 0f, false);
                }
#endif
            }

            return ikSnapshot;
        }

        private enum LegSide
        {
            Left,
            Right,
        }
    }
}