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
    internal class/*was_struct*/ HIKArmSolver
    {
        private const float InsideSwitchingMul = 2;
        private readonly HIKAvatarDefinition definition;
        private readonly quaternion _reorienter;
        private readonly float _upperRightLength;
        private readonly float _lowerRightLength;
        private readonly float _upperLeftLength;
        private readonly float _lowerLeftLength;
        private readonly float _shoulderRightLength;
        private readonly float _shoulderLeftLength;

        public HIKArmSolver(HIKAvatarDefinition definition, quaternion reorienter)
        {
            if (!definition.isInitialized) throw new InvalidOperationException("definition must be initialized before instantiating the solver");
            
            this.definition = definition;
            _reorienter = reorienter;

            _upperRightLength = math.distance(definition.refPoseHiplativePos[(int)HIKBodyBones.RightUpperArm], definition.refPoseHiplativePos[(int)HIKBodyBones.RightLowerArm]);
            _lowerRightLength = math.distance(definition.refPoseHiplativePos[(int)HIKBodyBones.RightLowerArm], definition.refPoseHiplativePos[(int)HIKBodyBones.RightHand]);
            _upperLeftLength = math.distance(definition.refPoseHiplativePos[(int)HIKBodyBones.LeftUpperArm], definition.refPoseHiplativePos[(int)HIKBodyBones.LeftLowerArm]);
            _lowerLeftLength = math.distance(definition.refPoseHiplativePos[(int)HIKBodyBones.LeftLowerArm], definition.refPoseHiplativePos[(int)HIKBodyBones.LeftHand]);
            
            _shoulderRightLength = math.distance(definition.refPoseHiplativePos[(int)HIKBodyBones.RightShoulder], definition.refPoseHiplativePos[(int)HIKBodyBones.RightUpperArm]);
            _shoulderLeftLength = math.distance(definition.refPoseHiplativePos[(int)HIKBodyBones.LeftShoulder], definition.refPoseHiplativePos[(int)HIKBodyBones.LeftUpperArm]);
        }

        public HIKSnapshot Solve(HIKObjective objective, HIKSnapshot ikSnapshot, bool debugDrawSolver)
        {
            var scale = math.length(objective.providedLossyScale) / math.length(definition.capturedWithLossyScale);
            
            // TODO: Add the ability for the solver to derive a lower arm sub-effector based on (the lower arm effector????? and) a L/R plane effector,
            // describing the intersection of two planes (a line) where the elbow joint could rest on. If the hand is at a fixed position, then the solution is the intersection between
            // a circle and the plane. The circle is described by the bend point rotating around the axis defined by the root pos and the objective pos.
            if (objective.solveRightArm) ikSnapshot = SolveArm(ikSnapshot, objective, ArmSide.Right, objective.rightHandTargetWorldPosition, objective.rightHandTargetWorldRotation, scale, debugDrawSolver);
            if (objective.solveLeftArm) ikSnapshot = SolveArm(ikSnapshot, objective, ArmSide.Left, objective.leftHandTargetWorldPosition, objective.leftHandTargetWorldRotation, scale, debugDrawSolver);
            return ikSnapshot;
        }

        private HIKSnapshot SolveArm(HIKSnapshot ikSnapshot, HIKObjective objective, ArmSide side, float3 originalObjectivePos, quaternion originalObjectiveRot, float scale, bool debugDrawSolver)
        {
            var rootBone = side == ArmSide.Right ? HIKBodyBones.RightUpperArm : HIKBodyBones.LeftUpperArm;
            var midBone = side == ArmSide.Right ? HIKBodyBones.RightLowerArm : HIKBodyBones.LeftLowerArm;
            var tipBone = side == ArmSide.Right ? HIKBodyBones.RightHand : HIKBodyBones.LeftHand;
            
            var rootPos = ikSnapshot.absolutePos[(int)rootBone];

            var upperLength = (side == ArmSide.Right ? _upperRightLength : _upperLeftLength) * scale;
            var lowerLength = (side == ArmSide.Right ? _lowerRightLength : _lowerLeftLength) * scale;
            
            // TODO: Handle HasUpperChest
            var chestReference = ikSnapshot.absoluteRot[(int)HIKBodyBones.Chest];
            var chestUpwards = math.mul(chestReference, math.right());

            if (objective.useShoulder > 0f)
            {
                // Shoulder
                var shoulderBone = side == ArmSide.Right ? HIKBodyBones.RightShoulder : HIKBodyBones.LeftShoulder;
                var shoulderLength = (side == ArmSide.Right ? _shoulderRightLength : _shoulderLeftLength) * scale;
                
                // TODO: Calculate the directional distance based on the shoulder length maximum extension in the given direction
                /*
                        - There are at least two defects:
                          - The shoulder angle at rest may be already angled, so at the maximum angle, it might pull the arm backwards.
                          - The extension distance is based on the shoulder length, but it doesn't multiply it by a function of the maximum angle.
                 */
                var directionalRelativeDistance = shoulderLength / (upperLength + lowerLength);
                
                var chestFrontwards = math.mul(chestReference, math.down());
                var prospectiveDirection = math.normalize(originalObjectivePos - rootPos);
                var frontwardness = math.clamp(math.dot(prospectiveDirection, chestFrontwards), 0f, 1f);
                var upwardness = math.clamp(math.dot(prospectiveDirection, chestUpwards), 0f, 1f);

                var extensionInfluence = objective.useShoulder * math.clamp(math.unlerp(0.7f, 1f + directionalRelativeDistance, math.length(originalObjectivePos - rootPos) / (lowerLength + upperLength)), 0f, 1f);
                var shoulderInfluenceFrontward = frontwardness * extensionInfluence;
                var shoulderInfluenceUpward = upwardness * extensionInfluence;
                if (shoulderInfluenceFrontward > 0 || shoulderInfluenceUpward > 0)
                {
                    var forwardRotation = quaternion.AxisAngle(math.right(), math.radians((side == ArmSide.Right ? -1f : 1f) * 60f * shoulderInfluenceFrontward * objective.shoulderForwardAngleMultiplier));
                    var upwardRotation = quaternion.AxisAngle(math.up(), math.radians((side == ArmSide.Right ? -1f : 1f) * 60f * shoulderInfluenceUpward * objective.shoulderUpwardAngleMultiplier));
                    var rotationToApply = math.mul(forwardRotation, upwardRotation);
                    ikSnapshot.ApplyRelativeRotation(shoulderBone, definition, rotationToApply);
                
                    ikSnapshot.ReevaluatePosition(rootBone, definition, scale);
                    var prevRootPos = rootPos;
                    rootPos = ikSnapshot.absolutePos[(int)rootBone];

#if UNITY_EDITOR && true
                    if (debugDrawSolver)
                    {
                        MbusUtil.DrawArrow(prevRootPos, rootPos, Color.magenta, 0f, false, chestUpwards);
                        Debug.DrawLine(ikSnapshot.absolutePos[(int)shoulderBone], rootPos, Color.yellow, 0f, false);
                    }
#endif
                }
            }

            // Corrections
            var TODO_STRADDLING_IS_FALSE = false;
            var objectivePos = HIKTwoBoneAlgorithms.ApplyCorrections(originalObjectivePos, TODO_STRADDLING_IS_FALSE, rootPos, upperLength, lowerLength, out var distanceType, objective.armStruggleStart, objective.armStruggleEnd, debugDrawSolver);

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
                
                var outwards = math.mul(chestReference, side == ArmSide.Right ? math.back() : math.forward());
                var handSource = math.mul(originalObjectiveRot, math.left());
                var palmDirection = math.mul(originalObjectiveRot, side == ArmSide.Right ? math.down() : math.up());

                var isOutwards = math.dot(outwards, -handSource);
                var isPalmUp = math.dot(chestUpwards, palmDirection);
                var isInside = math.clamp(math.smoothstep(0f, 1f, math.dot(-outwards, math.normalize(objectivePos - rootPos) * InsideSwitchingMul)), -1f, 1f);

#if UNITY_EDITOR && true
                if (debugDrawSolver)
                {
                    Debug.DrawLine(rootPos , rootPos + chestUpwards * isOutwards * 0.1f, Color.red, 0f, false);
                    Debug.DrawLine(rootPos + outwards * 0.01f, rootPos + outwards * 0.01f + chestUpwards * isPalmUp * 0.1f, Color.green, 0f, false);
                    Debug.DrawLine(rootPos + outwards * 0.02f, rootPos + outwards * 0.02f + chestUpwards * isInside * 0.1f, Color.blue, 0f, false);
                }
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
            (objectivePos, bendPointPos) = HIKTwoBoneAlgorithms.SolveBendPoint(rootPos, objectivePos, originalObjectiveRot, upperLength, lowerLength, TODO_STRADDLING_IS_FALSE, TODO_NO_STRADDLING_POSITION, distanceType, bendDirection, debugDrawSolver);

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
            ikSnapshot.ReevaluatePosition(rootBone, definition, scale);
            ikSnapshot.ReevaluatePosition(midBone, definition, scale);
            ikSnapshot.ReevaluatePosition(tipBone, definition, scale);

            return ikSnapshot;
        }

        private enum ArmSide
        {
            Left,
            Right,
        }
    }
}