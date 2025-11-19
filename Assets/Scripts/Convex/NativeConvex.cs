using System;
using System.Collections;
using System.Collections.Generic;
using Convex.DataStructures;
using Unity.Burst;
using Unity.Mathematics;
using UnityEngine;

[BurstCompile]
public struct NativeConvex : IEquatable<NativeConvex>,IComparable<NativeConvex>,IDisposable
{
    public int Id;
    public NativeHull Hull;
    public RigidTransform Transform;
    public float3 Scale;
    
    public NativeConvex(int id,NativeHull hull,RigidTransform rigidTransform,float3 scale)
    {
        Id = id;
        Hull = hull;
        Transform = rigidTransform;
        Scale = scale;
    }

    public bool Equals(NativeConvex other)
    {
        return Id == other.Id;
    }
    
    public override bool Equals(object obj)
    {
        return obj is NativeConvex other && other.Equals(this);
    }

    public int CompareTo(NativeConvex other)
    {
        return Id.CompareTo(other.Id);
    }
    
    public override int GetHashCode()
    {
        return Id;
    }

    public bool IsCreated()
    {
        return Hull.IsCreated();
    }

    public void Dispose()
    {
        Hull.Dispose();
    }
}
