using System;
using System.Collections.Generic;
using System.Linq;
using Convex.DataStructures;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace Convex
{
    public static class ConvexConstructor
    {
        public static NativeConvex CreatConvex(DetectionBody inBody)
        {
            NativeHull hull = CreateHull(inBody);

            return new NativeConvex(
                inBody.GetInstanceID(),
                hull,
                new RigidTransform(inBody.transform.rotation,inBody.transform.position),
                inBody.scale
            );
        }
        
        private static NativeHull CreateHull(DetectionBody inBody)
        {
            var mesh = inBody.MeshFilter;
            if (mesh && mesh.sharedMesh)
            {
                return ConvexConstructor.CreateNativeHull(mesh.sharedMesh);
            }
            throw new InvalidOperationException($"无法从游戏对象'{inBody?.name}'创建凸包");
        }
        
        private static NativeHull CreateNativeHull(Mesh mesh)
        {
            var Vertices = mesh.vertices;
            var Indices = mesh.triangles;
            NativeHull result = new NativeHull();

            float3[] vertices ;
            vertices = Vertices.Select(RoundVertex).ToArray();
            var edgeMap = new Dictionary<Tuple<int, int>, int>();
            //写入顶点数据
            SetHullVertices(ref result, vertices);
            //整理写入半边数据
            SetHullHalfEdges(ref result,ref edgeMap,Indices);
            //写入平面数据
            SetHullPlanes(ref result, ref edgeMap,Indices);
            
            return result;
        }

        private static void SetHullVertices(ref NativeHull hull, float3[] vertices)
        {
            hull.VertexCount = vertices.Length;
            hull.Vertices = new NativeArray<NativeVertex>(hull.VertexCount,Allocator.Persistent);
            for (int i = 0; i < hull.VertexCount; i++)
            {
                var v = hull.Vertices[i];
                v.Position = vertices[i];
                hull.Vertices[i] = v; 
            }
        }

        private static void SetHullHalfEdges(ref NativeHull hull,ref Dictionary<Tuple<int, int>, int> edgeMap,int[] indices)
        {
            int faceCount = indices.Length / 3;
            int halfEdgeCount = faceCount * 3;
            hull.Edges = new NativeArray<NativeHalfEdge>(halfEdgeCount, Allocator.Persistent);

            // 用于 twin 匹配：key=(start,end)，value=halfEdgeIndex

            int halfEdgeIndex = 0;
            for (int f = 0; f < faceCount; f++)
            {
                int i0 = indices[f * 3];
                int i1 = indices[f * 3 + 1];
                int i2 = indices[f * 3 + 2];

                // 三条半边索引
                int he0 = halfEdgeIndex++;
                int he1 = halfEdgeIndex++;
                int he2 = halfEdgeIndex++;

                // 依次设置半边数据
                hull.Edges[he0] = new NativeHalfEdge { StartVertex = i0, BelongFace = f, NextHalfedge = he1, PrevHalfedge = he2 , TwinHalfedge = -1};
                hull.Edges[he1] = new NativeHalfEdge { StartVertex = i1, BelongFace = f, NextHalfedge = he2, PrevHalfedge = he0 , TwinHalfedge = -1};
                hull.Edges[he2] = new NativeHalfEdge { StartVertex = i2, BelongFace = f, NextHalfedge = he0, PrevHalfedge = he1 , TwinHalfedge = -1};

                // 记录每条边的有向索引，用于之后匹配 twin
                edgeMap[Tuple.Create(i0, i1)] = he0;
                edgeMap[Tuple.Create(i1, i2)] = he1;
                edgeMap[Tuple.Create(i2, i0)] = he2;
            }

            // 第二遍：查找反向边并建立 Twin 关系
            foreach (var kvp in edgeMap)
            {
                var edge = kvp.Key;
                var start = edge.Item1;
                var end = edge.Item2;
                var keyReverse = Tuple.Create(end, start);

                if (edgeMap.TryGetValue(keyReverse, out int twinIndex))
                {
                    var he = hull.Edges[kvp.Value];
                    he.TwinHalfedge = twinIndex;
                    hull.Edges[kvp.Value] = he;
                }
            }

            hull.EdgeCount = halfEdgeCount;
        }

        private static void SetHullPlanes(ref NativeHull hull,ref Dictionary<Tuple<int, int>, int> edgeMap,int[] indices)
        {
            int PlanesCount = indices.Length / 3;
            hull.Planes = new NativeArray<NativePlane>(PlanesCount, Allocator.Persistent);
            float3 centroid;
            for (int i = 0; i < PlanesCount; i++)
            {
                int i0 = indices[i*3];
                int i1 = indices[i * 3 + 1];
                int i2 = indices[i * 3 + 2];
                
                float3 v0 = hull.Vertices[i0].Position;
                float3 v1 = hull.Vertices[i1].Position;
                float3 v2 = hull.Vertices[i2].Position;
                centroid = (v0 + v1 + v2) / 3;
                float3 edge1 = v1 - v0;
                float3 edge2 = v2 - v0;
                float3 normal = math.normalize(math.cross(edge1, edge2));
                float Offset = math.dot(centroid, normal);

                int firstHalfEdge;
                var tuple = new Tuple<int, int>(i0, i1);
                if (edgeMap.ContainsKey(tuple))
                {
                    firstHalfEdge = edgeMap[tuple];
                }
                else
                {
                    throw new InvalidOperationException($"无法找到面的起始边");
                }
                
                hull.Planes[i] = new NativePlane(normal,Offset,firstHalfEdge);
            }

            hull.PlaneCount = PlanesCount;
        }

        
        private static float3 RoundVertex(Vector3 v)
        {
            return new float3(
                (float)System.Math.Round(v.x, 3),
                (float)System.Math.Round(v.y, 3),
                (float)System.Math.Round(v.z, 3));
        }
    }
}