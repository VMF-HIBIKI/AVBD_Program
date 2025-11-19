using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;

[BurstCompile]
public struct SortJob : IJob
{
    public NativeArray<ulong> MortonCodes;
    public NativeArray<int> ObjectIndices;

    public void Execute()
    {
        var pairs = new NativeArray<KeyValuePair>(MortonCodes.Length, Allocator.Temp);

        for (int i = 0; i < MortonCodes.Length; i++)
        {
            pairs[i] = new KeyValuePair
            {
                Key = MortonCodes[i],
                Value = ObjectIndices[i]
            };
        }

        // 排序
        pairs.Sort(new KeyComparer());

        // 写回结果
        for (int i = 0; i < MortonCodes.Length; i++)
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