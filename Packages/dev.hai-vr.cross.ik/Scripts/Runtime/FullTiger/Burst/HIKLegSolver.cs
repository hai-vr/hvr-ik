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
    [BurstCompile]
    internal struct HIKLegSolver
    {
        private readonly HIKAvatarDefinition definition;
        private readonly quaternion _reorienter;
        private readonly AnimationCurve _curve;

        public HIKLegSolver(HIKAvatarDefinition definition, quaternion reorienter)
        {
            if (!definition.isInitialized) throw new InvalidOperationException("definition must be initialized before instantiating the solver");
            
            this.definition = definition;
            _reorienter = reorienter;
            _curve = new AnimationCurve(
                new Keyframe(0f, 0f, 0f, 2f),
                new Keyframe(0.2f, 0.63f, 1f, 1f),
                new Keyframe(0.5f, 0.86f, 0.5f, 0.5f),
                new Keyframe(1f, 1f, 0f, 0f)
            );
        }

        public HIKSnapshot Solve(HIKObjective objective, HIKSnapshot ikSnapshot)
        {
            if (objective.solveRightLeg) ikSnapshot = SolveLeg(ikSnapshot, objective, LegSide.Right, objective.rightFootTargetWorldPosition, objective.rightFootTargetWorldRotation, objective.useStraddlingRightLeg, objective.groundedStraddlingRightLegWorldPosition, objective.groundedStraddlingRightLegWorldRotation);
            if (objective.solveLeftLeg) ikSnapshot = SolveLeg(ikSnapshot, objective, LegSide.Left, objective.leftFootTargetWorldPosition, objective.leftFootTargetWorldRotation, objective.useStraddlingLeftLeg, objective.groundedStraddlingLeftLegWorldPosition, objective.groundedStraddlingLeftLegWorldRotation);
            return ikSnapshot;
        }

        private HIKSnapshot SolveLeg(HIKSnapshot ikSnapshot, HIKObjective objective, LegSide side, float3 originalObjectivePos, quaternion originalObjectiveRot, bool useStraddlingLeg, float3 groundedStraddlingLegWorldPosition, quaternion groundedStraddlingLegWorldRotation)
        {
            var objectivePos = originalObjectivePos;
            
            var rootBone = side == LegSide.Right ? HIKBodyBones.RightUpperLeg : HIKBodyBones.LeftUpperLeg;
            var midBone = side == LegSide.Right ? HIKBodyBones.RightLowerLeg : HIKBodyBones.LeftLowerLeg;
            var tipBone = side == LegSide.Right ? HIKBodyBones.RightFoot : HIKBodyBones.LeftFoot;
            
            var rootPos = ikSnapshot.absolutePos[(int)rootBone];

            var upperLength = math.distance(definition.refPoseHiplativePos[(int)rootBone], definition.refPoseHiplativePos[(int)midBone]);
            var lowerLength = math.distance(definition.refPoseHiplativePos[(int)midBone], definition.refPoseHiplativePos[(int)tipBone]);
            var totalLength = upperLength + lowerLength;
            var minimumDistance = math.abs(upperLength - lowerLength);
            
            // Apply correction when the target is too far
            bool isMaximumDistance;
            var distance = math.distance(rootPos, objectivePos);
            if (!useStraddlingLeg && distance >= totalLength * objective.legStruggleStart)
            {
                var lerpAmount = math.clamp(math.unlerp(totalLength * objective.legStruggleStart, totalLength * objective.legStruggleEnd, distance), 0f, 1f);
                var calculatedLength = math.lerp(totalLength * objective.legStruggleStart, totalLength, lerpAmount);
                objectivePos = rootPos + math.normalize(objectivePos - rootPos) * calculatedLength;
#if UNITY_EDITOR && true
                Debug.DrawLine(objectivePos, originalObjectivePos, Color.magenta, 0f, false);
#endif
                isMaximumDistance = calculatedLength >= totalLength;
            }
            else
            {
                isMaximumDistance = false;
            }
            
            // Apply correction when the target is practically unreachable
            bool isMinimumDistance;
            if (!useStraddlingLeg && distance < minimumDistance)
            {
                objectivePos = rootPos + math.normalize(objectivePos - rootPos) * minimumDistance;
#if UNITY_EDITOR && true
                Debug.DrawLine(originalObjectivePos, originalObjectivePos + math.up() * 0.1f, Color.magenta, 0f, false);
                Debug.DrawLine(objectivePos, originalObjectivePos + math.up() * 0.1f, Color.magenta, 0f, false);
#endif
                isMinimumDistance = true;
            }
            else
            {
                isMinimumDistance = false;
            }
            
            var hipReference = ikSnapshot.absoluteRot[(int)HIKBodyBones.Hips];
            var bendDirection = LegBendHeuristics();

            float3 LegBendHeuristics()
            {
                return math.normalize(math.mul(originalObjectiveRot, math.down() + math.left()));
            }
            
            // Solve
            bool isTooTight = false;
            float3 bendPointPos;
            if (useStraddlingLeg)
            {
                bendPointPos = rootPos + math.normalize(groundedStraddlingLegWorldPosition - rootPos) * upperLength;
                var prevObjectivePos = objectivePos;
                objectivePos = bendPointPos + math.normalize(objectivePos - bendPointPos) * lowerLength;

#if UNITY_EDITOR && true
                Debug.DrawLine(bendPointPos, groundedStraddlingLegWorldPosition, Color.magenta, 0f, false);
                Debug.DrawLine(prevObjectivePos, objectivePos, Color.magenta, 0f, false);
#endif
            }
            else if (!isMaximumDistance && !isMinimumDistance)
            {
                var toTip = objectivePos - rootPos;
                var toTipLength = math.length(toTip);
                
                // Law of cosines
                var angleRad = math.acos((toTipLength * toTipLength + upperLength * upperLength - lowerLength * lowerLength) / (2 * toTipLength * upperLength));
                // Ratio rule
                var toMidpointLength = math.cos(angleRad) * upperLength;
                var downDistance = math.sin(angleRad) * upperLength;

                var toMidpoint = math.normalize(toTip) * toMidpointLength;
                var bendDirectionStraightened = MbusGeofunctions.Straighten(math.normalize(bendDirection), toTip);
                bendPointPos = rootPos + toMidpoint + bendDirectionStraightened * downDistance;

                var v0 = math.mul(originalObjectiveRot, math.right());
                var v1 = math.normalize(bendPointPos - objectivePos);
                isTooTight = math.dot(v0, v1) > 0.01f;
            }
            else
            {
                if (isMinimumDistance)
                {
                    bendPointPos = rootPos + math.normalize(objectivePos - rootPos) * lowerLength;
                }
                else
                {
                    var toTip = objectivePos - rootPos;
                    var toMidpoint = toTip * upperLength / totalLength;
                    bendPointPos = rootPos + toMidpoint;
                }
            }

#if UNITY_EDITOR && true
            Debug.DrawLine(rootPos, objectivePos, Color.cyan, 0f, false);
            Debug.DrawLine(rootPos, bendPointPos, Color.yellow, 0f, false);
            Debug.DrawLine(bendPointPos, objectivePos, isTooTight ? Color.red : Color.yellow, 0f, false);
#endif

            ikSnapshot.absoluteRot[(int)rootBone] = math.mul(
                quaternion.LookRotationSafe(bendPointPos - rootPos, math.normalize(math.cross(math.mul(hipReference, math.forward()), bendPointPos - rootPos))),
                _reorienter
            );
            ikSnapshot.absoluteRot[(int)midBone] = math.mul(
                quaternion.LookRotationSafe(objectivePos - bendPointPos, math.normalize(math.cross(math.mul(originalObjectiveRot, math.back()), objectivePos - bendPointPos))),
                _reorienter
            );
            ikSnapshot.absoluteRot[(int)tipBone] = originalObjectiveRot;
            ikSnapshot.ReevaluatePosition(rootBone, definition);
            ikSnapshot.ReevaluatePosition(midBone, definition);
            ikSnapshot.ReevaluatePosition(tipBone, definition);

            return ikSnapshot;
        }

        private enum LegSide
        {
            Left,
            Right,
        }
    }
}