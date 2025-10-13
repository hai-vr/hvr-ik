using System;
using Unity.Burst;
using Unity.Mathematics;

namespace HVR.IK.FullTiger
{
    /// Given a definition and an objective, solves a pose into a snapshot.
    /// There is no dependency on the transform system beyond this point.
    /// Use Unity.Mathematics wherever applicable.
    [BurstCompile]
    internal struct HIKSolver
    {
        private readonly HIKSpineSolver _spineSolver;
        private readonly HIKArmSolver _armSolver;
        private readonly HIKLegSolver _legSolver;
        
        public HIKSolver(HIKAvatarDefinition definition)
        {
            if (!definition.isInitialized) throw new InvalidOperationException("definition must be initialized before instantiating the solver");

            var reorienter = MbusGeofunctions.FromToOrientation(math.forward(), math.right(), math.up(), -math.up());
            _spineSolver = new HIKSpineSolver(definition, reorienter);
            _armSolver = new HIKArmSolver(definition, reorienter);
            _legSolver = new HIKLegSolver(definition, reorienter);
        }

        public HIKSnapshot Solve(HIKObjective objective, HIKSnapshot ikSnapshot)
        {
            if (objective.solveSpine) ikSnapshot = _spineSolver.Solve(objective, ikSnapshot);
            
            // We need to solve the legs before the arms to support virtually parenting the hand effector to a bone of the leg.
            ikSnapshot = _legSolver.Solve(objective, ikSnapshot);
            RewriteObjectiveToAccountForHandSelfParenting(ikSnapshot, objective.selfParentRightHandNullable, ref objective.rightHandTargetWorldPosition, ref objective.rightHandTargetWorldRotation);
            RewriteObjectiveToAccountForHandSelfParenting(ikSnapshot, objective.selfParentLeftHandNullable, ref objective.leftHandTargetWorldPosition, ref objective.leftHandTargetWorldRotation);
            ikSnapshot = _armSolver.Solve(objective, ikSnapshot);
            return ikSnapshot;
        }

        private void RewriteObjectiveToAccountForHandSelfParenting(HIKSnapshot ikSnapshot, HIKSelfParenting objectiveSelfParentHandNullable, ref float3 pos, ref quaternion rot)
        {
            if (objectiveSelfParentHandNullable is { } parent && parent.use > 0f)
            {
                var parentBone = (int)parent.bone;
                var trs = math.mul(
                    float4x4.TRS(ikSnapshot.absolutePos[parentBone], ikSnapshot.absoluteRot[parentBone], new float3(1, 1, 1)),
                    float4x4.TRS(parent.relPosition, parent.relRotation, new float3(1, 1, 1))
                );
                pos = math.lerp(pos, trs.c3.xyz, parent.use);
                rot = math.slerp(rot, new quaternion(trs), parent.use);
            }
        }
    }
    
    [BurstCompile]
    internal struct HIKObjective
    {
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

        // FIXME: Switching to structs for burst makes these no longer nullable
        internal HIKSelfParenting selfParentLeftHandNullable;
        internal HIKSelfParenting selfParentRightHandNullable;
    }

    [BurstCompile]
    internal struct HIKSelfParenting
    {
        public float use;
        public HIKBodyBones bone;
        public float3 relPosition;
        public quaternion relRotation;
    }
}