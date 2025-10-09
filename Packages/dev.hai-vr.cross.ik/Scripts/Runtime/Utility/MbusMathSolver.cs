using Unity.Mathematics;
using UnityEngine;

namespace HVR.IK.FullTiger
{
    internal static class MbusMathSolver
    {
        /// Performs a single Fabrik iteration, which is made of several operations.
        ///
        /// For debugging purposes, the iteration can be interrupted in the middle of two operations.
        /// When a new operation is about to start but the operation counter has already reached the maximum value,
        /// further operations are not executed and this returns false.
        /// Returns true when the iteration has fully completed.
        public static bool Iterate(float3[] mutatedPointsIncludesRoot,
            float3 targetPos,
            float[] distances,
            float3 rootPos,
            ref int operationCounter,
            int maxOperationCount)
        {
            return Iterate(mutatedPointsIncludesRoot, targetPos, distances, rootPos, ref operationCounter, maxOperationCount, false, null);
        }

        /// Algorithm is based on:
        /// "Andreas Aristidou, Joan Lasenby, FABRIK: A fast, iterative solver for the Inverse Kinematics problem, Graphical Models, 73 (2011)"
        /// Some sections are not part of the original algorithm.
        public static bool Iterate(float3[] mutatedPointsIncludesRoot,
            float3 targetPos,
            float[] distances,
            float3 rootPos,
            ref int operationCounter,
            int maxOperationCount,
            bool supportsRepulsors,
            float3[] repulsorsNullable)
        {
            mutatedPointsIncludesRoot[mutatedPointsIncludesRoot.Length - 1] = targetPos;
            if (++operationCounter >= maxOperationCount) return false;
            for (var k = mutatedPointsIncludesRoot.Length - 2; k >= 1; k--)
            {
                mutatedPointsIncludesRoot[k] = ReachTowards(mutatedPointsIncludesRoot[k + 1], mutatedPointsIncludesRoot[k], distances[k]);
                var repulsion = float3.zero;
                if (supportsRepulsors)
                {
                    foreach (var repulsor in repulsorsNullable)
                    {
                        var threshold = 0.25f;
                        var distance = math.distance(mutatedPointsIncludesRoot[k], repulsor);
                        if (distance < threshold)
                        {
                            repulsion += math.normalize(mutatedPointsIncludesRoot[k] - repulsor) * ((threshold - distance) * 0.3f);
                        }
                    }
                }
                mutatedPointsIncludesRoot[k] += repulsion;
                if (++operationCounter >= maxOperationCount) return false;
            }

            mutatedPointsIncludesRoot[0] = rootPos;
            if (++operationCounter >= maxOperationCount) return false;
            for (var k = 1; k < mutatedPointsIncludesRoot.Length; k++)
            {
                mutatedPointsIncludesRoot[k] = ReachTowards(mutatedPointsIncludesRoot[k - 1], mutatedPointsIncludesRoot[k], distances[k - 1]);
                if (++operationCounter >= maxOperationCount) return k == mutatedPointsIncludesRoot.Length - 1;
            }

            return true;
        }

        private static float3 ReachTowards(float3 objective, float3 point, float distance)
        {
            return objective + math.normalize(point - objective) * distance;
        }

        public static bool SolveTriangle(float3[] mutatedThreePointsIncludesRoot,
            float3 targetPos,
            float[] distances,
            float3 rootPos,
            ref int operationCounter,
            int maxOperationCount)
        {
            // FIXME: This should be rewritten completely. Copied from an older project of mine.
            
            var first = mutatedThreePointsIncludesRoot[0];
            var second = mutatedThreePointsIncludesRoot[1];
            var third = mutatedThreePointsIncludesRoot[2];

            var upperArmDistance = distances[0];

            var totalArmDistance = upperArmDistance + distances[1];
            var longshotDistance = math.distance(rootPos, targetPos);
            var actualDistanceToReach = math.min(totalArmDistance, longshotDistance);

            var realTargetPos = rootPos + math.normalize(targetPos - rootPos) * actualDistanceToReach;
            var normalPlane = math.normalize(math.cross(first - second, third - second));
            var tangentPlane = math.normalize(math.cross(normalPlane, realTargetPos - rootPos));
            var ratio = upperArmDistance / totalArmDistance;
            var midpoint = math.lerp(rootPos, realTargetPos, ratio);

            var solveTri = ratio * actualDistanceToReach;
            var solvedSecond = midpoint + (upperArmDistance > solveTri ? (tangentPlane * math.sqrt(upperArmDistance * upperArmDistance - solveTri * solveTri)) : float3.zero);

            mutatedThreePointsIncludesRoot[0] = rootPos;
            mutatedThreePointsIncludesRoot[1] = solvedSecond;
            mutatedThreePointsIncludesRoot[2] = realTargetPos;

            if (++operationCounter >= maxOperationCount) return false;
            return true;
        }

        public static bool SolveStraddling(float3[] mutatedThreePointsIncludesRoot,
            float3 targetPos,
            float[] distances,
            float3 rootPos,
            ref int operationCounter,
            int maxOperationCount,
            float3 groundedStraddlingPos)
        {
            mutatedThreePointsIncludesRoot[0] = rootPos;
            var rootToGrounded = math.normalize(groundedStraddlingPos - rootPos);
            var middlePos = rootPos + rootToGrounded * distances[0];
            var middleToTarget = math.normalize(targetPos - middlePos);
            var realTargetPos = middlePos + middleToTarget * distances[1];
            mutatedThreePointsIncludesRoot[1] = middlePos;
            mutatedThreePointsIncludesRoot[2] = realTargetPos;

            if (++operationCounter >= maxOperationCount) return false;
            return true;
        }
    }
}
