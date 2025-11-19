using System;
using Unity.Collections;
using Unity.Mathematics;

namespace Convex.DataStructures
{
    public struct NativeHull : IDisposable
    {
        public int VertexCount;
        public int PlaneCount;
        public int EdgeCount;
        
        public NativeArray<NativeVertex> Vertices;
        public NativeArray<NativeHalfEdge> Edges;
        public NativeArray<NativePlane> Planes;

        public bool IsCreated()
        {
            return Vertices.IsCreated && Edges.IsCreated && Planes.IsCreated;
        }
        
        public void Dispose()
        {
            if (IsCreated())
            {
                Vertices.Dispose();
                Edges.Dispose();
                Planes.Dispose();
            }
        }
        
        public float3 GetSupportWorld(RigidTransform transform, float3 worldDir,float3 scale)
        {
            int index = GetSupportIndex(worldDir, transform);
            float3 localPoint = Vertices[index].Position;
            float3 scaledLocalPoint = localPoint * scale; 
            return math.transform(transform, scaledLocalPoint); // local -> world
        }


        public int GetSupportIndex(float3 direction,RigidTransform transform)
        {
            float3 localDir = math.mul(math.inverse(transform.rot), direction);
            int index = 0;
            float max = math.dot(localDir, Vertices[index].Position);
            for (int i = 1; i < VertexCount; ++i)
            {
                float dot = math.dot(localDir, Vertices[i].Position);
                if (dot > max)
                {
                    index = i;
                    max = dot;
                }
            }

            return index;
        }
    }
}