using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

[BurstCompile]
public struct MeshToAABBJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float3> Positions; // 每个物体的位置
    [ReadOnly] public NativeArray<float3> Scales;
    [ReadOnly] public NativeArray<Bounds> LocalBounds; // 每个Mesh的局部AABB
    [WriteOnly] public NativeArray<AABB> WorldAABBs;
    public NativeArray<quaternion>  Rotations;

    public void Execute(int i)
    {
        var local = LocalBounds[i];
        float3 localMin = local.min;
        float3 localMax = local.max;
        float3 v0 = new float3(localMin.x, localMin.y, localMin.z);
        float3 v1 = new float3(localMin.x, localMin.y, localMax.z);
        float3 v2 = new float3(localMin.x, localMax.y, localMin.z);
        float3 v3 = new float3(localMin.x, localMax.y, localMax.z);
        float3 v4 = new float3(localMax.x, localMin.y, localMin.z);
        float3 v5 = new float3(localMax.x, localMin.y, localMax.z);
        float3 v6 = new float3(localMax.x, localMax.y, localMin.z);
        float3 v7 = new float3(localMax.x, localMax.y, localMax.z);
        var rotation = Rotations[i];
        var scale = Scales[i];
        var position = Positions[i];
        float3 worldMin = new float3(float.MaxValue);
        float3 worldMax = new float3(float.MinValue);
        UpdateMinMax(v0, scale, rotation, position, ref worldMin, ref worldMax);
        UpdateMinMax(v1, scale, rotation, position, ref worldMin, ref worldMax);
        UpdateMinMax(v2, scale, rotation, position, ref worldMin, ref worldMax);
        UpdateMinMax(v3, scale, rotation, position, ref worldMin, ref worldMax);
        UpdateMinMax(v4, scale, rotation, position, ref worldMin, ref worldMax);
        UpdateMinMax(v5, scale, rotation, position, ref worldMin, ref worldMax);
        UpdateMinMax(v6, scale, rotation, position, ref worldMin, ref worldMax);
        UpdateMinMax(v7, scale, rotation, position, ref worldMin, ref worldMax);
        WorldAABBs[i] = new AABB
        {
            Min = worldMin,
            Max = worldMax
        };
    }
    
    private static void UpdateMinMax(float3 localVertex, float3 scale, quaternion rotation, float3 position, ref float3 worldMin, ref float3 worldMax)
    {
        float3 scaled = localVertex * scale;          // 缩放
        float3 rotated = math.rotate(rotation, scaled); // 旋转
        float3 worldPos = rotated + position;         // 平移
        worldMin = math.min(worldMin, worldPos);      // 更新最小
        worldMax = math.max(worldMax, worldPos);      // 更新最大
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
