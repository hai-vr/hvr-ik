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

using System.Numerics;

namespace Tooling;

internal class HIKBendLookup
{
    private const int Divisions = 10;
    private const int Size = Divisions * 2 + 1;

    private readonly Vector3[][][] _lookupTable;

    private Func<(Vector3 handPos, Quaternion handRot), Vector3> _lookupFn;

    public HIKBendLookup()
    {
        _lookupTable = new Vector3[Size][][];
        for (var j = 0; j < Size; j++)
        {
            _lookupTable[j] = new Vector3[Size][];
            for (var i = 0; i < Size; i++) _lookupTable[j][i] = new Vector3[Size];
        }
    }

    public void SetLookupFunction(Func<(Vector3 handPos, Quaternion handRot), Vector3> handPosToBendPointFn)
    {
        _lookupFn = handPosToBendPointFn;
    }

    public void BakeLookupTable()
    {
        for (var i = 0; i < Size; i++)
        for (var j = 0; j < Size; j++)
        for (var k = 0; k < Size; k++)
        {
            var handPos = new Vector3(i / (Size - 1f) * 2 - 1, j / (Size - 1f) * 2 - 1, k / (Size - 1f) * 2 - 1);
            var bendPoint = _lookupFn((handPos, Quaternion.Identity));
            _lookupTable[i][j][k] = bendPoint;
        }
    }

    public List<Vector3> ExportLookupTable()
    {
        var results = new List<Vector3>();

        for (var i = 0; i < HIKBendLookup.Size; i++)
        for (var j = 0; j < HIKBendLookup.Size; j++)
        for (var k = 0; k < HIKBendLookup.Size; k++)
        {
            results.Add(_lookupTable[i][j][k]);
        }

        return results;
    }

    public Vector3 Lookup(Vector3 handPos)
    {
        var x = (handPos.X + 1f) * 0.5f * (Size - 1);
        var y = (handPos.Y + 1f) * 0.5f * (Size - 1);
        var z = (handPos.Z + 1f) * 0.5f * (Size - 1);

        x = Math.Clamp(x, 0f, Size - 1);
        y = Math.Clamp(y, 0f, Size - 1);
        z = Math.Clamp(z, 0f, Size - 1);

        var x0 = (int)Math.Floor(x);
        var y0 = (int)Math.Floor(y);
        var z0 = (int)Math.Floor(z);
        var x1 = Math.Min(x0 + 1, Size - 1);
        var y1 = Math.Min(y0 + 1, Size - 1);
        var z1 = Math.Min(z0 + 1, Size - 1);

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
        var c00 = Lerp(c000, c100, fx);
        var c01 = Lerp(c001, c101, fx);
        var c10 = Lerp(c010, c110, fx);
        var c11 = Lerp(c011, c111, fx);

        var c0 = Lerp(c00, c10, fy);
        var c1 = Lerp(c01, c11, fy);

        return Lerp(c0, c1, fz);
    }

    private static Vector3 Lerp(Vector3 a, Vector3 b, float t)
    {
        return a + (b - a) * t;
    }
}