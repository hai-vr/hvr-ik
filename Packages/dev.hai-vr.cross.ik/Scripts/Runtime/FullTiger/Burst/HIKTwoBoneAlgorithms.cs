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
    internal static class HIKTwoBoneAlgorithms
    {
        internal static float3 ApplyCorrections(float3 originalObjectivePos,
            bool useStraddlingLeg,
            float3 rootPos,
            float upperLength,
            float lowerLength,
            out HIKTwoBoneDistanceType distanceType,
            float struggleStart,
            float struggleEnd,
            bool debugDrawSolver)
        {
            var totalLength = upperLength + lowerLength;
            var minimumDistance = math.abs(upperLength - lowerLength);
            
            var objectivePos = originalObjectivePos;
            
            // Apply correction when the target is too far
            var distance = math.distance(rootPos, objectivePos);
            if (!useStraddlingLeg && distance >= totalLength * struggleStart)
            {
                float finalLength;
                if (struggleStart != struggleEnd)
                {
                    var lerpAmount = math.clamp(math.unlerp(totalLength * struggleStart, totalLength * struggleEnd, distance), 0f, 1f);
                    var calculatedLength = math.lerp(totalLength * struggleStart, totalLength, 1 - math.pow(1 - lerpAmount, 4f));
                    finalLength = calculatedLength;
                }
                else
                {
                    finalLength = totalLength;
                }
                objectivePos = rootPos + math.normalize(objectivePos - rootPos) * finalLength;
#if UNITY_EDITOR && true
                if (debugDrawSolver) Debug.DrawLine(objectivePos, originalObjectivePos, Color.magenta, 0f, false);
#endif
                distanceType = finalLength >= totalLength ? HIKTwoBoneDistanceType.MaximumDistance : HIKTwoBoneDistanceType.Regular;
            }
            else
            {
                // Apply correction when the target is practically unreachable
                if (!useStraddlingLeg && distance < minimumDistance)
                {
                    objectivePos = rootPos + math.normalize(objectivePos - rootPos) * minimumDistance;
#if UNITY_EDITOR && true
                    if (debugDrawSolver)
                    {
                        Debug.DrawLine(originalObjectivePos, originalObjectivePos + math.up() * 0.1f, Color.magenta, 0f, false);
                        Debug.DrawLine(objectivePos, originalObjectivePos + math.up() * 0.1f, Color.magenta, 0f, false);
                    }
#endif
                    distanceType = HIKTwoBoneDistanceType.MinimumDistance;
                }
                else
                {
                    distanceType = HIKTwoBoneDistanceType.Regular;
                }
            }

            return objectivePos;
        }

        public static (float3 objectivePos, float3 bendPointPos) SolveBendPoint(float3 rootPos,
            float3 objectivePos,
            quaternion objectiveRot,
            float upperLength,
            float lowerLength,
            bool useStraddlingMode,
            float3 straddlingWorldPosition,
            HIKTwoBoneDistanceType distanceType,
            float3 bendDirection, bool debugDrawSolver)
        {
            float3 bendPointPos;
            var totalLength = upperLength + lowerLength;
            
            bool isTooTight = false;
            if (useStraddlingMode)
            {
                bendPointPos = rootPos + math.normalize(straddlingWorldPosition - rootPos) * upperLength;
                var prevObjectivePos = objectivePos;
                objectivePos = bendPointPos + math.normalize(objectivePos - bendPointPos) * lowerLength;

#if UNITY_EDITOR && true
                if (debugDrawSolver)
                {
                    Debug.DrawLine(bendPointPos, straddlingWorldPosition, Color.magenta, 0f, false);
                    Debug.DrawLine(prevObjectivePos, objectivePos, Color.magenta, 0f, false);
                }
#endif
            }
            else
            {
                if (distanceType == HIKTwoBoneDistanceType.Regular)
                {
                    var toTip = objectivePos - rootPos;
                    var toTipLength = math.length(toTip);

                    // Law of cosines
                    var acosInput = (toTipLength * toTipLength + upperLength * upperLength - lowerLength * lowerLength) / (2 * toTipLength * upperLength);
                    if (acosInput <= -1f)
                    {
                        // When this happens, we're actually at a maximum distance, possibly due to imprecision.
                        var toMidpoint = toTip * upperLength / totalLength;
                        bendPointPos = rootPos + toMidpoint;
                    }
                    else if (acosInput >= 1f)
                    {
                        // When this happens, we're actually at a minimum distance, possibly due to imprecision.
                        bendPointPos = rootPos + math.normalize(objectivePos - rootPos) * lowerLength;
                    }
                    else
                    {
                        var angleRad = math.acos(acosInput);
                        // Ratio rule
                        var toMidpointLength = math.cos(angleRad) * upperLength;
                        var downDistance = math.sin(angleRad) * upperLength;

                        var toMidpoint = math.normalize(toTip) * toMidpointLength;
                        var bendDirectionStraightened = MbusGeofunctions.Straighten(math.normalize(bendDirection), toTip);
                        bendPointPos = rootPos + toMidpoint + bendDirectionStraightened * downDistance;

                        var v0 = math.mul(objectiveRot, math.right());
                        var v1 = math.normalize(bendPointPos - objectivePos);
                        isTooTight = math.dot(v0, v1) > 0.01f;
                    }
                }
                else if (distanceType == HIKTwoBoneDistanceType.MinimumDistance)
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
            if (debugDrawSolver)
            {
                Debug.DrawLine(rootPos, objectivePos, Color.cyan, 0f, false);
                Debug.DrawLine(rootPos, bendPointPos, Color.yellow, 0f, false);
                Debug.DrawLine(bendPointPos, objectivePos, isTooTight ? Color.red : Color.yellow, 0f, false);
            }
#endif
            return (objectivePos, bendPointPos);
        }

        public static AnimationCurve CreateStruggleCurve()
        {
            return new AnimationCurve(
                new Keyframe(0f, 0f, 0f, 2f),
                new Keyframe(0.2f, 0.63f, 1f, 1f),
                new Keyframe(0.5f, 0.86f, 0.5f, 0.5f),
                new Keyframe(1f, 1f, 0f, 0f)
            );
        }
    }

    internal enum HIKTwoBoneDistanceType
    {
        Regular,
        MaximumDistance,
        MinimumDistance
    }
}