using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;

[BurstCompile]
public struct SortJob : IJob
{
    public NativeArray<ulong> MortonCodes;
    public NativeArray<int> ObjectIndices;
    public int Count;

    public void Execute()
    {
        int n = Count;
        var pairs = new NativeArray<KeyValuePair>(n, Allocator.Temp);

        for (int i = 0; i < n; i++)
        {
            pairs[i] = new KeyValuePair
            {
                Key = MortonCodes[i],
                Value = ObjectIndices[i]
            };
        }

        pairs.Sort(new KeyComparer());

        for (int i = 0; i < n; i++)
        {
            MortonCodes[i] = pairs[i].Key;
            ObjectIndices[i] = pairs[i].Value;
        }

        pairs.Dispose();
    }

    private struct KeyValuePair : System.IComparable<KeyValuePair>
    {
        public ulong Key;
        public int Value;

        public int CompareTo(KeyValuePair other)
        {
            return Key.CompareTo(other.Key);
        }
    }

    private struct KeyComparer : IComparer<KeyValuePair>
    {
        public int Compare(KeyValuePair x, KeyValuePair y)
        {
            return x.Key.CompareTo(y.Key);
        }
    }
}
