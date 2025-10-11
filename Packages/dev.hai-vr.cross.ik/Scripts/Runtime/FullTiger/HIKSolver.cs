using System;
using Unity.Mathematics;

namespace HVR.IK.FullTiger
{
    /// Given a definition and an objective, solves a pose into a snapshot.
    /// There is no dependency on the transform system beyond this point.
    /// Use Unity.Mathematics wherever applicable.
    internal class HIKSolver
    {
        private readonly HIKSpineSolver _spineSolver;
        private readonly HIKArmSolver _armSolver;
        private readonly HIKLegSolver _legSolver;

        public HIKSolver(HIKAvatarDefinition definition, HIKSnapshot ikSnapshot)
        {
            if (!definition.isInitialized) throw new InvalidOperationException("definition must be initialized before instantiating the solver");

            var reorienter = MbusGeofunctions.FromToOrientation(math.forward(), math.right(), math.up(), -math.up());
            _spineSolver = new HIKSpineSolver(definition, ikSnapshot, reorienter);
            _armSolver = new HIKArmSolver(definition, ikSnapshot, reorienter);
            _legSolver = new HIKLegSolver(definition, ikSnapshot, reorienter);
        }

        public void Solve(HIKObjective objective)
        {
            if (objective.solveSpine) _spineSolver.Solve(objective);
            _armSolver.Solve(objective);
            _legSolver.Solve(objective);
        }
    }
    
    internal class HIKObjective
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
        
        public float useChest;
        internal float3 chestTargetWorldPosition;
        internal quaternion chestTargetWorldRotation;
        internal float alsoUseChestToMoveNeck;
        
        internal bool headAlignmentMattersMore;
        internal bool allowContortionist;
        
        public bool useStraddlingLeftLeg;
        public bool useStraddlingRightLeg;
        internal float3 groundedStraddlingLeftLegWorldPosition;
        internal quaternion groundedStraddlingLeftLegWorldRotation;
        internal float3 groundedStraddlingRightLegWorldPosition;
        internal quaternion groundedStraddlingRightLegWorldRotation;
        
        public bool solveSpine;
        public bool solveLeftLeg;
        public bool solveRightLeg;
        public bool solveLeftArm;
        public bool solveRightArm;
    }
}