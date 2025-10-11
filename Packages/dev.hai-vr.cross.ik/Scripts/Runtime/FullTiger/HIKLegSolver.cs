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
    internal class HIKLegSolver
    {
        private const float InsideSwitchingMul = 2;
        private readonly HIKAvatarDefinition definition;
        private readonly HIKSnapshot ikSnapshot;
        private readonly quaternion _reorienter;
        
        public HIKLegSolver(HIKAvatarDefinition definition, HIKSnapshot ikSnapshot, quaternion reorienter)
        {
            if (!definition.isInitialized) throw new InvalidOperationException("definition must be initialized before instantiating the solver");
            
            this.definition = definition;
            this.ikSnapshot = ikSnapshot;
            _reorienter = reorienter;
        }

        public void Solve(HIKObjective objective)
        {
            SolveArm(objective, LegSide.Right, objective.rightFootTargetWorldPosition, objective.rightFootTargetWorldRotation);
            SolveArm(objective, LegSide.Left, objective.leftFootTargetWorldPosition, objective.leftFootTargetWorldRotation);
        }

        private void SolveArm(HIKObjective objective, LegSide side, float3 originalObjectivePos, quaternion originalObjectiveRot)
        {
            var objectivePos = originalObjectivePos;
            
            var rootBone = side == LegSide.Right ? HumanBodyBones.RightUpperLeg : HumanBodyBones.LeftUpperLeg;
            var midBone = side == LegSide.Right ? HumanBodyBones.RightLowerLeg : HumanBodyBones.LeftLowerLeg;
            var tipBone = side == LegSide.Right ? HumanBodyBones.RightFoot : HumanBodyBones.LeftFoot;
            
            var rootPos = ikSnapshot.absolutePos[(int)rootBone];

            var upperLength = math.distance(definition.refPoseHiplativePos[(int)rootBone], definition.refPoseHiplativePos[(int)midBone]);
            var lowerLength = math.distance(definition.refPoseHiplativePos[(int)midBone], definition.refPoseHiplativePos[(int)tipBone]);
            var totalLength = upperLength + lowerLength;
            
            // Apply correction when the target is too far
            bool isMaximumDistance;
            if (math.distance(rootPos, objectivePos) >= totalLength)
            {
                objectivePos = rootPos + math.normalize(objectivePos - rootPos) * totalLength;
                Debug.DrawLine(objectivePos, originalObjectivePos, Color.magenta, 0f, false);
                isMaximumDistance = true;
            }
            else
            {
                isMaximumDistance = false;
            }
            
            var hipReference = ikSnapshot.absoluteRot[(int)HumanBodyBones.Hips];
            var bendDirection = LegBendHeuristics();

            float3 LegBendHeuristics()
            {
                return math.normalize(math.mul(originalObjectiveRot, math.down() + math.left()));
            }
            
            // Solve
            bool isTooTight = false;
            float3 bendPointPos;
            if (!isMaximumDistance)
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
                var toTip = objectivePos - rootPos;
                var toMidpoint = toTip * upperLength / totalLength;
                bendPointPos = rootPos + toMidpoint;
            }

            Debug.DrawLine(rootPos, objectivePos, Color.cyan, 0f, false);
            Debug.DrawLine(rootPos, bendPointPos, Color.yellow, 0f, false);
            Debug.DrawLine(bendPointPos, objectivePos, isTooTight ? Color.red : Color.yellow, 0f, false);

            var hipDown = math.mul(hipReference, math.down());
            ikSnapshot.absoluteRot[(int)rootBone] = math.mul(
                quaternion.LookRotationSafe(bendPointPos - rootPos, MbusUtil.LerpDot(hipDown, hipDown, math.mul(hipReference, math.right()), math.dot(hipDown, math.normalize(bendPointPos - rootPos)))),
                _reorienter
            );
            ikSnapshot.absoluteRot[(int)midBone] = math.mul(
                quaternion.LookRotationSafe(objectivePos - bendPointPos, math.mul(originalObjectiveRot, math.down())),
                _reorienter
            );
            ikSnapshot.absoluteRot[(int)tipBone] = originalObjectiveRot;
            ikSnapshot.ReevaluatePosition(rootBone, definition);
            ikSnapshot.ReevaluatePosition(midBone, definition);
            ikSnapshot.ReevaluatePosition(tipBone, definition);
        }

        private enum LegSide
        {
            Left,
            Right,
        }
    }
}