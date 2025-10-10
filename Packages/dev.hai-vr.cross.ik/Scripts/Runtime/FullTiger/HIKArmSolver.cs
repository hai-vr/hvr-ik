using System;
using Unity.Mathematics;
using UnityEngine;

namespace HVR.IK.FullTiger
{
    internal class HIKArmSolver
    {
        private readonly HIKAvatarDefinition definition;
        private readonly HIKSnapshot ikSnapshot;
        private readonly quaternion _reorienter;
        private readonly HIKSolver solver;
        
        public HIKArmSolver(HIKAvatarDefinition definition, HIKSnapshot ikSnapshot, quaternion reorienter, HIKSolver solver)
        {
            if (!definition.isInitialized) throw new InvalidOperationException("definition must be initialized before instantiating the solver");
            
            this.definition = definition;
            this.ikSnapshot = ikSnapshot;
            _reorienter = reorienter;
            this.solver = solver;
        }

        public void Solve(HIKObjective objective)
        {
            SolveRightArm(objective, ArmSide.Right, objective.rightHandTargetWorldPosition, objective.rightHandTargetWorldRotation);
        }

        private void SolveRightArm(HIKObjective objective, ArmSide side, float3 originalObjectivePos, quaternion originalObjectiveRot)
        {
            var objectivePos = originalObjectivePos;
            
            var rootBone = side == ArmSide.Right ? HumanBodyBones.RightUpperArm : HumanBodyBones.LeftUpperArm;
            var midBone = side == ArmSide.Right ? HumanBodyBones.RightLowerArm : HumanBodyBones.LeftLowerArm;
            var tipBone = side == ArmSide.Right ? HumanBodyBones.RightHand : HumanBodyBones.LeftHand;
            
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
            
            // TODO: Build the bend direction heuristics
            // TODO: Handle HasUpperChest
            var chestReference = ikSnapshot.absoluteRot[(int)HumanBodyBones.Chest];
            var bendDirection = math.mul(chestReference, math.left());
            
            // Solve
            bool isTooTight = false;
            float3 bendPointPos;
            if (!isMaximumDistance)
            {
                var toTip = objectivePos - rootPos;
                var toMidpoint = toTip * upperLength / totalLength;
                var toMidpointLength = math.length(toMidpoint);
                var downDistance = math.sqrt(upperLength * upperLength - toMidpointLength * toMidpointLength);
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

            // FIXME: Resolve twist for this. The twist is also a function of the hand rotation.
            float3 twist = bendDirection;
            ikSnapshot.absoluteRot[(int)rootBone] = math.mul(
                quaternion.LookRotationSafe(bendPointPos - rootPos, twist),
                _reorienter
            );
            ikSnapshot.absoluteRot[(int)midBone] = math.mul(
                quaternion.LookRotationSafe(objectivePos - bendPointPos, twist),
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