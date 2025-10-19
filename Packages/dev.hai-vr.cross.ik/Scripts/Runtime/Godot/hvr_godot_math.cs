#if !UNITY_2022_1_OR_NEWER //__GODOT

using System;
using Godot;
using float3 = Godot.Vector3;
using float4x4 = Godot.Transform3D;
using quaternion = Godot.Quaternion;

public class hvr_godot_math
{
    public static float3 right() { return new float3(1, 0, 0); }
    public static float3 left() { return new float3(-1, 0, 0); }
    public static float3 up() { return new float3(0, 1, 0); }
    public static float3 down() { return new float3(0, -1, 0); }
    public static float3 forward() { return new float3(0, 0, -1); }
    public static float3 back() { return new float3(0, 0, 1); }
    
    public static float distance(float3 a, float3 b)
    {
        return a.DistanceTo(b);
    }

    public static float length(float3 vec)
    {
        return vec.Length();
    }

    public static float3 normalize(float3 vec)
    {
        return vec.Normalized();
    }

    public static float dot(float3 a, float3 b)
    {
        return a.Dot(b);
    }

    public static float3 cross(float3 a, float3 b)
    {
        return a.Cross(b);
    }
    
    public static float3 mul(quaternion rotation, float3 direction)
    {
        return rotation * direction;
    }

    public static quaternion mul(quaternion ra, quaternion rb)
    {
        return ra * rb;
    }

    public static float4x4 mul(float4x4 a, float4x4 b)
    {
        return a * b;
    }

    public static float clamp(float val, float start, float end)
    {
        return Mathf.Clamp(val, start, end);
    }

    public static float abs(float val)
    {
        return Mathf.Abs(val);
    }

    public static float pow(float a, float e)
    {
        return Mathf.Pow(a, e);
    }

    public static float3 lerp(float3 a, float3 b, float v)
    {
        return a.Lerp(b, v);
    }

    public static float lerp(float a, float b, float v)
    {
        return Mathf.Lerp(a, b, v);
    }

    public static float unlerp(float start, float end, float v)
    {
        return (v - start) / (end - start);
    }

    public static quaternion slerp(quaternion a, quaternion b, float v)
    {
        return a.Slerp(b, v);
    }

    public static float radians(float degrees)
    {
        return Mathf.DegToRad(degrees);
    }

    public static float degrees(float radians)
    {
        return Mathf.RadToDeg(radians);
    }

    public static float acos(float f)
    {
        return Mathf.Acos(f);
    }

    public static float smoothstep(float start, float end, float v)
    {
        var t = clamp((v - start) / (end - start), 0f, 1f);
        return t * t * (3f - 2f * t);
    }

    public static float cos(float angleRad)
    {
        return Mathf.Cos(angleRad);
    }

    public static float sin(float angleRad)
    {
        return Mathf.Sin(angleRad);
    }

    public static quaternion inverse(quaternion v)
    {
        return v.Inverse();
    }
}

#endif