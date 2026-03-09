using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

[BurstCompile]
public struct MeshToAABBJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float3> Positions;
    [ReadOnly] public NativeArray<float3> Scales;
    [ReadOnly] public NativeArray<Bounds> LocalBounds;
    [ReadOnly] public NativeArray<quaternion> Rotations;
    [WriteOnly] public NativeArray<AABB> WorldAABBs;

    public void Execute(int i)
    {
        var local = LocalBounds[i];
        float3 center  = (float3)local.center;
        float3 extents = (float3)local.extents;
        float3 scale   = Scales[i];

        float3 sc = center * scale;
        float3 se = extents * math.abs(scale);

        quaternion rot = Rotations[i];
        float3x3 rm = new float3x3(rot);
        float3 worldExtents = math.abs(rm.c0) * se.x
                            + math.abs(rm.c1) * se.y
                            + math.abs(rm.c2) * se.z;
        float3 worldCenter = math.rotate(rot, sc) + Positions[i];

        WorldAABBs[i] = new AABB
        {
            Min = worldCenter - worldExtents,
            Max = worldCenter + worldExtents
        };
    }
}

[BurstCompile]
public struct AABB
{
    public float3 Min;
    public float3 Max;

    public float3 Center => (Min + Max) * 0.5f;
    public float3 Size => Max - Min;
}
