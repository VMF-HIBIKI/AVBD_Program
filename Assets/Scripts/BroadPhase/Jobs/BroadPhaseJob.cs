using System.Threading;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

public struct LBVHNode
{
    public AABB bounds;
    public int left;
    public int right;
    public int parent;
    public int objectIdx; // >= 0 leaf, -1 internal
}

[BurstCompile]
public struct InitLBVHJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<AABB> aabbs;
    [ReadOnly] public NativeArray<int> sortedIndices;
    [ReadOnly] public int internalCount;
    [WriteOnly] [NativeDisableParallelForRestriction]
    public NativeArray<LBVHNode> nodes;

    public void Execute(int i)
    {
        if (i < internalCount)
        {
            nodes[i] = new LBVHNode
                { left = -1, right = -1, parent = -1, objectIdx = -1 };
        }
        else
        {
            int oi = sortedIndices[i - internalCount];
            nodes[i] = new LBVHNode
            {
                bounds = aabbs[oi],
                left = -1, right = -1,
                parent = -1, objectIdx = oi
            };
        }
    }
}

/// Karras 2012 parallel radix-tree construction.
/// Operates on n-1 internal nodes (indices 0..n-2).
/// Leaf nodes are at indices n-1 .. 2n-2.
[BurstCompile]
public struct BuildRadixTreeJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<ulong> sortedCodes;
    [ReadOnly] public int leafCount;
    [NativeDisableContainerSafetyRestriction]
    public NativeArray<LBVHNode> nodes;

    public unsafe void Execute(int i)
    {
        int n = leafCount;
        int ic = n - 1;
        var p = (LBVHNode*)nodes.GetUnsafePtr();

        int dL = Delta(i, i - 1);
        int dR = Delta(i, i + 1);
        int d = dR > dL ? 1 : -1;

        int dMin = Delta(i, i - d);
        int lmax = 2;
        while (Delta(i, i + lmax * d) > dMin) lmax <<= 1;

        int l = 0;
        for (int t = lmax >> 1; t >= 1; t >>= 1)
        {
            int probe = i + (l + t) * d;
            if (probe >= 0 && probe < n && Delta(i, probe) > dMin)
                l += t;
        }

        int j = i + l * d;
        int first = math.min(i, j);
        int last  = math.max(i, j);
        int dNode = Delta(first, last);

        int s = first;
        int t2 = last - first;
        do
        {
            t2 = (t2 + 1) >> 1;
            if (s + t2 < last && Delta(first, s + t2) > dNode)
                s += t2;
        } while (t2 > 1);

        int gamma = s;
        int leftChild  = gamma == first      ? gamma + ic     : gamma;
        int rightChild = gamma + 1 == last   ? gamma + 1 + ic : gamma + 1;

        p[i].left      = leftChild;
        p[i].right     = rightChild;
        p[i].objectIdx = -1;
        p[leftChild].parent  = i;
        p[rightChild].parent = i;
    }

    int Delta(int i, int j)
    {
        if (j < 0 || j >= leafCount) return -1;
        ulong xr = sortedCodes[i] ^ sortedCodes[j];
        if (xr == 0)
            return 64 + math.lzcnt((uint)((uint)i ^ (uint)j));
        uint hi = (uint)(xr >> 32);
        return hi != 0 ? math.lzcnt(hi) : 32 + math.lzcnt((uint)xr);
    }
}

/// Bottom-up bounding-box computation.
/// Each leaf walks up; an atomic counter ensures the second child
/// to arrive computes the union and continues upward.
[BurstCompile]
public struct ComputeBVHBoundsJob : IJobParallelFor
{
    [ReadOnly] public int leafCount;
    [NativeDisableContainerSafetyRestriction]
    public NativeArray<LBVHNode> nodes;
    [NativeDisableContainerSafetyRestriction]
    public NativeArray<int> counters;

    public unsafe void Execute(int leafIdx)
    {
        int ic = leafCount - 1;
        var p = (LBVHNode*)nodes.GetUnsafePtr();
        var c = (int*)counters.GetUnsafePtr();

        int cur = p[ic + leafIdx].parent;
        while (cur >= 0)
        {
            if (Interlocked.Increment(ref c[cur]) == 1) return;

            int lc = p[cur].left;
            int rc = p[cur].right;
            p[cur].bounds = new AABB
            {
                Min = math.min(p[lc].bounds.Min, p[rc].bounds.Min),
                Max = math.max(p[lc].bounds.Max, p[rc].bounds.Max)
            };
            cur = p[cur].parent;
        }
    }
}

/// Stack-based tree traversal. Each leaf descends from root,
/// pruning branches whose AABB doesn't overlap.
/// Reports pairs where objectIdxA < objectIdxB to avoid duplicates.
[BurstCompile]
public struct LBVHTraversalJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<LBVHNode> nodes;
    [ReadOnly] public int leafCount;
    public NativeList<int2>.ParallelWriter pairs;

    public unsafe void Execute(int leafIdx)
    {
        int ic = leafCount - 1;
        int myNode = ic + leafIdx;
        AABB myBox = nodes[myNode].bounds;
        int myObj  = nodes[myNode].objectIdx;

        int* stack = stackalloc int[256];
        int top = 0;
        stack[top++] = 0;

        while (top > 0)
        {
            int cur = stack[--top];
            var nd = nodes[cur];
            CheckChild(nd.left,  myBox, myObj, stack, ref top);
            CheckChild(nd.right, myBox, myObj, stack, ref top);
        }
    }

    unsafe void CheckChild(int idx, AABB myBox, int myObj,
                            int* stack, ref int top)
    {
        var nd = nodes[idx];
        if (myBox.Min.x > nd.bounds.Max.x || myBox.Max.x < nd.bounds.Min.x ||
            myBox.Min.y > nd.bounds.Max.y || myBox.Max.y < nd.bounds.Min.y ||
            myBox.Min.z > nd.bounds.Max.z || myBox.Max.z < nd.bounds.Min.z)
            return;

        if (nd.objectIdx >= 0)
        {
            if (nd.objectIdx > myObj)
                pairs.AddNoResize(new int2(myObj, nd.objectIdx));
        }
        else
        {
            stack[top++] = idx;
        }
    }
}
