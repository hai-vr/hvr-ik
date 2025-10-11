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

using Unity.Mathematics;
using UnityEngine;

namespace HVR.IK.FullTiger
{
    /// Contains any general purpose functions to facilitate geometric operations.
    internal static class MbusGeofunctions
    {
        /**
         * Creates a rotation which rotates from fromDirection to toDirection, and the direction of fromUpwards is oriented to the direction of toUpwards.
         * The upwards direction will be a best effort to match as they don't need to be perpendicular to the direction.
         *
         * Usually, you can use it to reorient an object to match another object.
         *
         * This function is an attempt to combine both LookRotation and FromToDirection:
         * - FromToDirection does not enforce an upwards axis, only one axis needs to match.
         * - LookRotation can difficult to grasp when trying to rotate an object to face a direction, but the object's forward isn't the direction that needs to face it.
         */
        public static quaternion FromToOrientation(float3 fromDirection, float3 toDirection, float3 fromUpwards,
            float3 toUpwards)
        {
            var fromRotation = quaternion.LookRotationSafe(fromDirection, fromUpwards);
            var toRotation = quaternion.LookRotationSafe(toDirection, toUpwards);
            return math.mul(toRotation, math.inverse(fromRotation));
        }

        /**
         * Provides the distance between a line and a point.
         */
        public static float DistanceBetweenLineAndPoint(float3 point, float3 linePoint, float3 lineVector)
        {
            return math.distance(Untested__ProjectOnPlane(point, lineVector), Untested__ProjectOnPlane(linePoint, lineVector));
        }
        
        private static float3 Untested__ProjectOnPlane(float3 vector, float3 planeNormal)
        {
            var normalizedPlaneNormal = math.normalize(planeNormal);
            return vector - math.dot(vector, normalizedPlaneNormal) * normalizedPlaneNormal;
        }

        /**
         * Extract the position and rotation out of a TRS matrix.
         */
        public static void ToPosRotV3(float4x4 matrix, out float3 pos, out quaternion rot)
        {
            var vector4 = matrix.c3;
            pos = vector4.xyz;
            rot = quaternion.LookRotationSafe(matrix.c2.xyz, matrix.c1.xyz);
        }

        /**
         * Returns a normalized vector perpendicular to the axis.
         * That vector is straightened from a vector that is:
         * - similar to the first vector if that first vector is perpendicular to the axis,
         * - similar to the second vector if that first vector is not perpendicular to the axis.
         */
        public static float3 Screwdriver(float3 first, float3 second, float3 axis)
        {
            return math.normalize(Straighten(ImpracticalScrewdriver(first, second, axis), axis));
        }

        /**
         * Returns a vector that is:
         * - similar to the first vector if that first vector is perpendicular to the axis,
         * - similar to the second vector if that first vector is not perpendicular to the axis.
         */
        private static float3 ImpracticalScrewdriver(float3 first, float3 second, float3 axis)
        {
            return math.lerp(first, second, math.dot(axis, first));
        }

        /**
         * Makes the vector to be straightened perpendicular to the axis.
         */
        public static float3 Straighten(float3 toStraighten, float3 onAxis)
        {
            return math.normalize(math.cross(math.cross(onAxis, toStraighten), onAxis)) * math.length(toStraighten);
        }

        /**
         * Given two signed angles sampled in close proximity representing a change in angle,
         * provide a new rotor with that change in angle.
         *
         * It assumes the consecutive signed angles does not represent a change exceeding a threshold set to 90 degrees.
         */
        public static float Rotor(float currentSignedAngle, float previousSignedAngle, float previousRotor)
        {
            return previousRotor + DiffSignedAngle(currentSignedAngle, previousSignedAngle);
        }

        private static float DiffSignedAngle(float currentSignedAngle, float previousSignedAngle)
        {
            // Went too negative
            if (currentSignedAngle > 90 && previousSignedAngle < -90)
            {
                // return (-180 - previousSignedAngle) - (180 - currentSignedAngle);
                return currentSignedAngle - previousSignedAngle - 360;
            }

            // Went too positive
            if (currentSignedAngle < -90 && previousSignedAngle > 90)
            {
                // return 180 - previousSignedAngle + (currentSignedAngle + 180);
                return currentSignedAngle - previousSignedAngle + 360;
            }

            return currentSignedAngle - previousSignedAngle;
        }

        public static float3 Slerp(float3 a, float3 b, float t)
        {
            return Vector3.Slerp(a, b, t); // FIXME: Vector3.Slerp doesn't use unity mathematics.
        }

        public static float3 LerpDot(float3 whenMinusOne, float3 whenZero, float3 whenOne, float dot)
        {
            if (dot >= 0)
            {
                return math.lerp(whenZero, whenOne, dot);
            }
            else
            {
                return math.lerp(whenMinusOne, whenZero, dot + 1);
            }
        }
    }
}
