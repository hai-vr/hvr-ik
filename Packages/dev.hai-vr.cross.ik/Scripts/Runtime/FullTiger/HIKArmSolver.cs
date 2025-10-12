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
    internal class HIKArmSolver
    {
        private const float InsideSwitchingMul = 2;
        private readonly HIKAvatarDefinition definition;
        private readonly HIKSnapshot ikSnapshot;
        private readonly quaternion _reorienter;
        
        public HIKArmSolver(HIKAvatarDefinition definition, HIKSnapshot ikSnapshot, quaternion reorienter)
        {
            if (!definition.isInitialized) throw new InvalidOperationException("definition must be initialized before instantiating the solver");
            
            this.definition = definition;
            this.ikSnapshot = ikSnapshot;
            _reorienter = reorienter;
        }

        public void Solve(HIKObjective objective)
        {
            // TODO: Add the ability for the solver to derive a lower arm sub-effector based on (the lower arm effector????? and) a L/R plane effector,
            // describing the intersection of two planes (a line) where the elbow joint could rest on. If the hand is at a fixed position, then the solution is the intersection between
            // a circle and the plane. The circle is described by the bend point rotating around the axis defined by the root pos and the objective pos.
            if (objective.solveRightArm) SolveArm(objective, ArmSide.Right, objective.rightHandTargetWorldPosition, objective.rightHandTargetWorldRotation);
            if (objective.solveLeftArm) SolveArm(objective, ArmSide.Left, objective.leftHandTargetWorldPosition, objective.leftHandTargetWorldRotation);
        }

        private void SolveArm(HIKObjective objective, ArmSide side, float3 originalObjectivePos, quaternion originalObjectiveRot)
        {
            var objectivePos = originalObjectivePos;
            
            var rootBone = side == ArmSide.Right ? HumanBodyBones.RightUpperArm : HumanBodyBones.LeftUpperArm;
            var midBone = side == ArmSide.Right ? HumanBodyBones.RightLowerArm : HumanBodyBones.LeftLowerArm;
            var tipBone = side == ArmSide.Right ? HumanBodyBones.RightHand : HumanBodyBones.LeftHand;
            
            var rootPos = ikSnapshot.absolutePos[(int)rootBone];

            var upperLength = math.distance(definition.refPoseHiplativePos[(int)rootBone], definition.refPoseHiplativePos[(int)midBone]);
            var lowerLength = math.distance(definition.refPoseHiplativePos[(int)midBone], definition.refPoseHiplativePos[(int)tipBone]);
            var totalLength = upperLength + lowerLength;
            var minimumDistance = math.abs(upperLength - lowerLength);
            
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
            
            // Apply correction when the target is practically unreachable
            bool isMinimumDistance;
            if (math.distance(rootPos, objectivePos) < minimumDistance)
            {
                objectivePos = rootPos + math.normalize(objectivePos - rootPos) * minimumDistance;
                Debug.DrawLine(originalObjectivePos, originalObjectivePos + math.up() * 0.1f, Color.magenta, 0f, false);
                Debug.DrawLine(objectivePos, originalObjectivePos + math.up() * 0.1f, Color.magenta, 0f, false);
                isMinimumDistance = true;
            }
            else
            {
                isMinimumDistance = false;
            }
            
            // TODO: Handle HasUpperChest
            var chestReference = ikSnapshot.absoluteRot[(int)HumanBodyBones.Chest];
            var bendDirection = ArmBendHeuristics();

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
                
                Debug.DrawLine(rootPos , rootPos + chestUpwards * isOutwards * 0.1f, Color.red, 0f, false);
                Debug.DrawLine(rootPos + outwards * 0.01f, rootPos + outwards * 0.01f + chestUpwards * isPalmUp * 0.1f, Color.green, 0f, false);
                Debug.DrawLine(rootPos + outwards * 0.02f, rootPos + outwards * 0.02f + chestUpwards * isInside * 0.1f, Color.blue, 0f, false);
                
                var chestSource = math.mul(chestReference, math.left());
                var chestSourceBendingOutwards = math.normalize(chestSource + outwards * math.clamp(isInside, 0f, 1f));
                var step2 = MbusGeofunctions.LerpDot(handSource, handSource, chestSourceBendingOutwards, isPalmUp);
                var step3 = MbusGeofunctions.LerpDot(step2, step2, chestSourceBendingOutwards, isOutwards);
                var regular = MbusGeofunctions.LerpDot(step3, step3, chestSourceBendingOutwards, isInside);
                
                return useBend <= 0 ? regular : math.lerp(regular, directedBend, useBend);
            }
            
            // Solve
            bool isTooTight = false;
            float3 bendPointPos;
            if (!isMaximumDistance && !isMinimumDistance)
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
            else if (isMinimumDistance)
            {
                bendPointPos = rootPos + math.normalize(objectivePos - rootPos) * lowerLength;
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

            var twistBase = math.mul(chestReference, math.left());
            ikSnapshot.absoluteRot[(int)rootBone] = math.mul(
                quaternion.LookRotationSafe(bendPointPos - rootPos, side == ArmSide.Right ? twistBase : -twistBase),
                _reorienter
            );
            ikSnapshot.absoluteRot[(int)midBone] = math.mul(
                quaternion.LookRotationSafe(objectivePos - bendPointPos, math.mul(originalObjectiveRot, side == ArmSide.Right ? math.forward() : math.back())),
                _reorienter
            );
            ikSnapshot.absoluteRot[(int)tipBone] = originalObjectiveRot;
            ikSnapshot.ReevaluatePosition(rootBone, definition);
            ikSnapshot.ReevaluatePosition(midBone, definition);
            ikSnapshot.ReevaluatePosition(tipBone, definition);
        }

        private enum ArmSide
        {
            Left,
            Right,
        }
    }
}