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
using System.Collections.Generic;
#if UNITY_2020_1_OR_NEWER //__NOT_GODOT
using Unity.Mathematics;
using UnityEngine;
#else //__iff HVR_IS_GODOT
using float3 = Godot.Vector3;
using float2 = Godot.Vector2;
using float4x4 = Godot.Transform3D;
using quaternion = Godot.Quaternion;
using math = hvr_godot_math;
#endif

namespace HVR.IK.FullTiger
{
    internal class HIKBendLookup
    {
        private const int Divisions = 10;
        private const int Size = (Divisions * 2) + 1;

        private readonly float3[][][] _lookupTable;
        
        private Func<(float3 handPos, quaternion handRot), float3> _lookupFn;

        public HIKBendLookup()
        {
            _lookupTable = new float3[Size][][];
            for (var j = 0; j < Size; j++)
            {
                _lookupTable[j] = new float3[Size][];
                for (var i = 0; i < Size; i++)
                {
                    _lookupTable[j][i] = new float3[Size];
                }
            }
        }

        public void SetLookupFunction(Func<(float3 handPos, quaternion handRot), float3> handPosToBendPointFn)
        {
            _lookupFn = handPosToBendPointFn;
        }

        public void BakeLookupTable()
        {
            for (var i = 0; i < Size; i++)
            {
                for (var j = 0; j < Size; j++)
                {
                    for (var k = 0; k < Size; k++)
                    {
                        var handPos = new Vector3(i / (Size - 1f) * 2 - 1, j / (Size - 1f) * 2 - 1, k / (Size - 1f) * 2 - 1);
                        var bendPoint = _lookupFn((handPos, quaternion.identity));
                        _lookupTable[i][j][k] = bendPoint;
                    }
                }
            }
        }

        public float3 Lookup(float3 handPos)
        {
            var x = (handPos.x + 1f) * 0.5f * (Size - 1);
            var y = (handPos.y + 1f) * 0.5f * (Size - 1);
            var z = (handPos.z + 1f) * 0.5f * (Size - 1);

            x = math.clamp(x, 0f, Size - 1);
            y = math.clamp(y, 0f, Size - 1);
            z = math.clamp(z, 0f, Size - 1);

            var x0 = (int)math.floor(x);
            var y0 = (int)math.floor(y);
            var z0 = (int)math.floor(z);
            var x1 = math.min(x0 + 1, Size - 1);
            var y1 = math.min(y0 + 1, Size - 1);
            var z1 = math.min(z0 + 1, Size - 1);

            var fx = x - x0;
            var fy = y - y0;
            var fz = z - z0;

            var c000 = _lookupTable[x0][y0][z0];
            var c001 = _lookupTable[x0][y0][z1];
            var c010 = _lookupTable[x0][y1][z0];
            var c011 = _lookupTable[x0][y1][z1];
            var c100 = _lookupTable[x1][y0][z0];
            var c101 = _lookupTable[x1][y0][z1];
            var c110 = _lookupTable[x1][y1][z0];
            var c111 = _lookupTable[x1][y1][z1];

            // Trilinear interpolation
            var c00 = math.lerp(c000, c100, fx);
            var c01 = math.lerp(c001, c101, fx);
            var c10 = math.lerp(c010, c110, fx);
            var c11 = math.lerp(c011, c111, fx);

            var c0 = math.lerp(c00, c10, fy);
            var c1 = math.lerp(c01, c11, fy);

            return math.lerp(c0, c1, fz);
        }

        public void ImportLookupTable(List<float3> lookupTable)
        {
            for (var index = 0; index < lookupTable.Count; index++)
            {
                var i = index / (Size * Size);
                var j = (index / Size) % Size;
                var k = index % Size;
                _lookupTable[i][j][k] = lookupTable[index];
            }
        }

        public float3 GetBendDirectionInWorldSpace__UsingLookupTable(ArmSide side, quaternion chestReference, float3 rootPos, float3 objectivePos, quaternion originalObjectiveRot, float totalArmLength)
        {
            HIKArmBendLookupSpace.ConvertToLookupSpace(side, chestReference, rootPos, objectivePos, originalObjectiveRot, totalArmLength, out var reoriented, out var objectivePosInLookupSpace, out var objectiveRotInLookupSpace);

            var bendDirectionInLookupSpace = Lookup(objectivePosInLookupSpace);
            
            HIKArmBendLookupSpace.ConvertDirectionToWorldSpace(side, bendDirectionInLookupSpace, reoriented, out var bendDirectionInWorldSpace);

            return bendDirectionInWorldSpace;
        }

        public float3 GetBendPositionInWorldSpace__UsingLookupTable(ArmSide side, quaternion chestReference, float3 rootPos, float3 objectivePos, quaternion originalObjectiveRot, float totalArmLength)
        {
            HIKArmBendLookupSpace.ConvertToLookupSpace(side, chestReference, rootPos, objectivePos, originalObjectiveRot, totalArmLength, out var reoriented, out var objectivePosInLookupSpace, out var objectiveRotInLookupSpace);

            var bendDirectionInLookupSpace = Lookup(objectivePosInLookupSpace);
            
            HIKArmBendLookupSpace.ConvertPositionToWorldSpace(side, rootPos, totalArmLength, bendDirectionInLookupSpace, reoriented, out var bendDirectionInWorldSpace);

            return bendDirectionInWorldSpace;
        }
    }

    internal static class HIKArmBendLookupSpace
    {
        private static readonly quaternion FromChestToLookupSpace = MbusGeofunctions.FromToOrientation(math.right(), math.back(), math.forward(), math.down());

        internal static void ConvertToLookupSpace(ArmSide side, quaternion chestReference, float3 rootPos, float3 objectivePos, quaternion originalObjectiveRot, float totalArmLength, out quaternion reoriented, out float3 objectivePosInLookupSpace, out quaternion objectiveRotInLookupSpace)
        {
            // ## lookupSpace:
            // handPos should be in a coordinate space:
            // - rotated so that Right points to the tip of the arm
            // - rotated so that Up points to the direction of the head
            // - centered at the upper arm root AFTER shoulder angle is applied,
            // - scaled so that maximum length of the arm (sum of upper arm length and lower arm length) equates to one.
            
            // FIXME: We need to reorient the chest reference first to align it with the lookup space
            reoriented = math.mul(chestReference, FromChestToLookupSpace);
            var reorientedInverse = math.inverse(reoriented);
            
            // (rootPos becomes zero)
            objectivePosInLookupSpace = math.mul(reorientedInverse, (objectivePos - rootPos) / totalArmLength);
            objectiveRotInLookupSpace = math.mul(reorientedInverse, originalObjectiveRot);
            
            if (side == ArmSide.Left)
            {
                // The lookup space only deals with the right hand, so we need to flip along the plane normal to the left--right axis at zero
                objectivePosInLookupSpace.x = -objectivePosInLookupSpace.x;
                var fwd = math.mul(objectiveRotInLookupSpace, math.forward());
                var up = math.mul(objectiveRotInLookupSpace, math.up());
                fwd.x = -fwd.x;
                up.x = -up.x;
                
                // The -up needs an explanation:
                // The up direction of the palm in objectiveRot represents the direction of the palm. When we flip X, we would get a
                // mirrored fwd and up vectors as expected, but the palm direction gets flipped as well. We don't want this as this compromises calculations later,
                // so we also reverse up so that the palm direction matches. Not sure if that explanation makes sense, but the effect is noticeable if you commend out this line below
                // and then place the hands on the forehead, the arm directions become incorrect.
                var reversedUp = -up;
                objectiveRotInLookupSpace = hvr_godot_helper_quaternion.LookRotationSafe(fwd, reversedUp);
            }
        }

        internal static void ConvertDirectionToWorldSpace(ArmSide side, float3 bendDirectionInLookupSpace, quaternion reoriented, out float3 bendDirectionInWorldSpace)
        {
            if (side == ArmSide.Left)
            {
                bendDirectionInLookupSpace.x = -bendDirectionInLookupSpace.x;
            }

            bendDirectionInWorldSpace = math.mul(reoriented, bendDirectionInLookupSpace);
        }

        internal static void ConvertPositionToWorldSpace(ArmSide side, float3 rootPos, float totalArmLength, float3 bendDirectionInLookupSpace, quaternion reoriented, out float3 bendDirectionInWorldSpace)
        {
            if (side == ArmSide.Left)
            {
                bendDirectionInLookupSpace.x = -bendDirectionInLookupSpace.x;
            }

            bendDirectionInWorldSpace = rootPos + math.mul(reoriented, bendDirectionInLookupSpace * totalArmLength);
        }
    }

    internal static class HIKArmBendDefaultHeuristics
    {
        internal static float3 GetBendDirectionInWorldSpace(ArmSide side, quaternion chestReference, float3 rootPos, float3 objectivePos, quaternion originalObjectiveRot, float totalArmLength)
        {
            HIKArmBendLookupSpace.ConvertToLookupSpace(side, chestReference, rootPos, objectivePos, originalObjectiveRot, totalArmLength, out var reoriented, out var objectivePosInLookupSpace, out var objectiveRotInLookupSpace);

            var bendDirectionInLookupSpace = GetBendDirectionInLookupSpace(objectivePosInLookupSpace, objectiveRotInLookupSpace);
            
            HIKArmBendLookupSpace.ConvertDirectionToWorldSpace(side, bendDirectionInLookupSpace, reoriented, out var bendDirectionInWorldSpace);
            
            return bendDirectionInWorldSpace;
        }

        internal static float3 GetBendDirectionInLookupSpace(float3 objectivePosInLookupSpace, quaternion objectiveRotInLookupSpace)
        {
            return GetBendDirectionInLookupSpace__Legacy(ArmSide.Right, objectivePosInLookupSpace, objectiveRotInLookupSpace);
        }

        private static float3 GetBendDirectionInLookupSpace__Legacy(ArmSide side, float3 objectivePosInLookupSpace, quaternion objectiveRotInLookupSpace)
        {
            var chestOutwards = side == ArmSide.Right ? math.right() : math.left();
            var chestUpwards = math.up();
            
            var handSource = math.mul(objectiveRotInLookupSpace, math.left());
            var palmDirection = math.mul(objectiveRotInLookupSpace, side == ArmSide.Right ? math.down() : math.up());

            var isOutwards = math.dot(chestOutwards, -handSource);
            var isPalmUp = math.dot(chestUpwards, palmDirection);
            var isInside = math.clamp(math.smoothstep(0f, 1f, math.dot(-chestOutwards, math.normalize(objectivePosInLookupSpace) * HIKArmSolver.InsideSwitchingMul)), -1f, 1f);

// #if UNITY_EDITOR && true
//             if (debugDrawSolver)
//             {
//                 Debug.DrawLine(rootPos , rootPos + chestUpwards * isOutwards * 0.1f, Color.red, 0f, false);
//                 Debug.DrawLine(rootPos + chestOutwards * 0.01f, rootPos + chestOutwards * 0.01f + chestUpwards * isPalmUp * 0.1f, Color.green, 0f, false);
//                 Debug.DrawLine(rootPos + chestOutwards * 0.02f, rootPos + chestOutwards * 0.02f + chestUpwards * isInside * 0.1f, Color.blue, 0f, false);
//             }
// #endif
            
            var chestSource = math.down();
            var chestSourceBendingOutwards = math.normalize(chestSource + chestOutwards * math.clamp(isInside, 0f, 1f));
            var step2 = MbusGeofunctions.LerpDot(handSource, handSource, chestSourceBendingOutwards, isPalmUp);
            var step3 = MbusGeofunctions.LerpDot(step2, step2, chestSourceBendingOutwards, isOutwards);
            var regular = MbusGeofunctions.LerpDot(step3, step3, chestSourceBendingOutwards, isInside);
            
            return regular;
        }
    }
}