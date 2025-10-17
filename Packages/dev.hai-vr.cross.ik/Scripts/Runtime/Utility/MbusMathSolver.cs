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

using Unity.Collections;
using Unity.Mathematics;

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
        public static bool Iterate(ref HIKSpineData<float3> mutatedPointsIncludesRoot,
            float3 targetPos,
            HIKSpineData<float> distances,
            float3 rootPos,
            ref int operationCounter,
            int maxOperationCount,
            float scale)
        {
            return Iterate(ref mutatedPointsIncludesRoot, targetPos, distances, rootPos, ref operationCounter, maxOperationCount, false, /*default,*/ scale);
        }

        /// Algorithm is based on:
        /// "Andreas Aristidou, Joan Lasenby, FABRIK: A fast, iterative solver for the Inverse Kinematics problem, Graphical Models, 73 (2011)"
        /// Some sections are not part of the original algorithm.
        public static bool Iterate(ref HIKSpineData<float3> mutatedPointsIncludesRoot,
            float3 targetPos,
            HIKSpineData<float> distances,
            float3 rootPos,
            ref int operationCounter,
            int maxOperationCount,
            bool supportsRepulsors,
            // HIKNotAnArray<float3> repulsorsNullable,
            float scale)
        {
            mutatedPointsIncludesRoot[mutatedPointsIncludesRoot.Length - 1] = targetPos;
            if (++operationCounter >= maxOperationCount) return false;
            for (var k = mutatedPointsIncludesRoot.Length - 2; k >= 1; k--)
            {
                mutatedPointsIncludesRoot[k] = ReachTowards(mutatedPointsIncludesRoot[k + 1], mutatedPointsIncludesRoot[k], distances[k] * scale);
                var repulsion = float3.zero;
                // if (supportsRepulsors)
                // {
                //     foreach (var repulsor in repulsorsNullable)
                //     {
                //         var threshold = 0.25f;
                //         var distance = math.distance(mutatedPointsIncludesRoot[k], repulsor);
                //         if (distance < threshold)
                //         {
                //             repulsion += math.normalize(mutatedPointsIncludesRoot[k] - repulsor) * ((threshold - distance) * 0.3f);
                //         }
                //     }
                // }
                mutatedPointsIncludesRoot[k] += repulsion;
                if (++operationCounter >= maxOperationCount) return false;
            }

            mutatedPointsIncludesRoot[0] = rootPos;
            if (++operationCounter >= maxOperationCount) return false;
            for (var k = 1; k < mutatedPointsIncludesRoot.Length; k++)
            {
                mutatedPointsIncludesRoot[k] = ReachTowards(mutatedPointsIncludesRoot[k - 1], mutatedPointsIncludesRoot[k], distances[k - 1] * scale);
                if (++operationCounter >= maxOperationCount) return k == mutatedPointsIncludesRoot.Length - 1;
            }

            return true;
        }

        private static float3 ReachTowards(float3 objective, float3 point, float distance)
        {
            return objective + math.normalize(point - objective) * distance;
        }
    }
}
