using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

[BurstCompile]
public struct MortonCodeJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<AABB> AABBs;
    [ReadOnly] public AABB SceneBounds;
    [ReadOnly] public int MaxDepth;

    [WriteOnly] public NativeArray<ulong> MortonCodes;
    [WriteOnly] public NativeArray<int> Levels;
    
    [ReadOnly] public ulong Granularity; // 2^MaxDepth - 1,颗粒度
    [ReadOnly] public ulong BitMask; // 用于Part1By2的基础掩码（低MaxDepth位为1）


    public void Execute(int i)
    {
        var box = AABBs[i];
        float3 size = box.Size;
        float maxExtent = math.cmax(size);
        float sceneSize = math.cmax(SceneBounds.Size);

        int level = 0;
        float nodeSize = sceneSize;
        while (level < MaxDepth && maxExtent < nodeSize * 0.5f)
        {
            nodeSize *= 0.5f;
            level++;
        }

        var center = box.Center;
        float3 norm = (center - SceneBounds.Min) / SceneBounds.Size;
        norm = math.clamp(norm, 0f, 1f);

        ulong morton = EncodeMorton3D(norm.x, norm.y, norm.z,MaxDepth);
        

        MortonCodes[i] = morton;
        Levels[i] = level;
    }

    private ulong EncodeMorton3D(float x, float y, float z , int depth)
    {
        // 使用动态Granularity转换为MaxDepth位整数
        ulong xx = Part1By2((ulong)(x * Granularity),depth);
        ulong yy = Part1By2((ulong)(y * Granularity),depth);
        ulong zz = Part1By2((ulong)(z * Granularity),depth);
        return (xx << 2) + (yy << 1) + zz;
    }

    private ulong Part1By2(ulong x , int depth)
    {
        // 使用动态掩码处理MaxDepth位输入
        x &= BitMask; // 只保留有效位
        if(depth>15) x = (x ^ (x << 32)) & 0x1F00000000FFFFUL;
        if(depth>7) x = (x ^ (x << 16)) & 0x1F0000FF0000FFUL;
        if(depth>3) x = (x ^ (x << 8)) & 0x100F00F00F00F00FUL;
        if(depth>1)x = (x ^ (x << 4)) & 0x10C30C30C30C30C3UL;
        if(depth>1)x = (x ^ (x << 2)) & 0x1249249249249249UL;
        return x;
    }
}