using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

[BurstCompile]
public struct BroadPhaseJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<AABB> AABBs;
    [ReadOnly] public NativeArray<ulong> MortonCodes;
    [ReadOnly] public NativeArray<int> ObjectIndices;
    [ReadOnly] public NativeArray<int> Levels;
    [ReadOnly] public int MaxDepth;
    [ReadOnly] public int Count;

    public NativeList<int2>.ParallelWriter CollisionPairs;

    public void Execute(int i)
    {
        
        if (i >= Count) return;
        int idA = ObjectIndices[i];
        var boxA = AABBs[idA];

        
        for (int j = i+1; j < Count; j++)
        {
            int idB = ObjectIndices[j];
            if (idA == idB) continue; // 避免自己碰自己
            //确定公共祖辈所在层级
            int publicParentLevel = math.min(Levels[idA], Levels[idB]);
            publicParentLevel = publicParentLevel-2 < 0 ? 0 : publicParentLevel - 2;
            //判断各自的公共祖辈层级是否是同一个节点,不是就跳过
            
            int shiftBits = 3 * (MaxDepth - publicParentLevel);
            ulong prefixA = MortonCodes[i] >> shiftBits;
            ulong prefixB = MortonCodes[j] >> shiftBits;
            if (prefixA != prefixB) continue;
            
            //前缀相同，再进行精确的AABB碰撞检测
            var boxB = AABBs[idB];
            
            if (Overlap(boxA, boxB))
                CollisionPairs.AddNoResize(new int2(idA, idB));
        }
    }

    private static bool Overlap(AABB a, AABB b)
    {
        return (a.Min.x <= b.Max.x && a.Max.x >= b.Min.x) &&
               (a.Min.y <= b.Max.y && a.Max.y >= b.Min.y) &&
               (a.Min.z <= b.Max.z && a.Max.z >= b.Min.z);
    }
}