using System;
using Convex.DataStructures;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace Collision
{
    [BurstCompile]
    public static class GJK_EPA
    {
        
        private const float Epsilon = 1e-5f;
        
        public static bool DetectedCollisionAndResolve(in NativeConvex convexA, in NativeConvex convexB,out EPAResult epaResult)
        {
            NativeList<SupportPoint> simplex = new NativeList<SupportPoint>(Allocator.Temp);
            bool isCollision = GJK_Slove(convexA, convexB, ref simplex);
            if (!isCollision)
            {
                simplex.Dispose();
                epaResult = default;
                return false;
            }
            EPA_Solve(convexA, convexB,ref simplex, out epaResult);
            simplex.Dispose();
            return true;
        }

        private static bool GJK_Slove(in NativeConvex convexA, in NativeConvex convexB,
            ref NativeList<SupportPoint> simplex)
        {
            const int MaxIters = 256;
            float3 dir = new float3(1,1,1);
            SupportPoint initial = Support(convexA, convexB, dir);
            dir = -initial.Point;
            for (int i = 0; i < MaxIters; i++)
            {
                SupportPoint newPoint = Support(convexA, convexB, dir);
                
                if (math.dot(newPoint.Point, dir) < 0)
                {
                    // 无法推进，说明不碰撞
                    return false;
                }

                simplex.Add(newPoint);

                if (NextSimplex(ref simplex, ref dir))
                    return true;
            }

            return false;
        }
        
        private static bool NextSimplex(ref NativeList<SupportPoint> simplex, ref float3 dir)
        {
            switch (simplex.Length)
            {
                case 2: return Line(ref simplex, ref dir);
                case 3: return Triangle(ref simplex, ref dir);
                case 4: return Tetrahedron(ref simplex, ref dir);
                default: return false;
            }
        }
        
        private static bool Line(ref NativeList<SupportPoint> simplex, ref float3 dir)
        {
            SupportPoint a = simplex[1];
            SupportPoint b = simplex[0];
            float3 ab = b.Point - a.Point;
            float3 ao = -a.Point;

            // dir 朝向原点正交于 AB
            dir = math.cross(math.cross(ab, ao), ab);
            if (math.lengthsq(dir) < Epsilon)
                dir = new float3(1, 0, 0);

            return false;
        }
        
        private static bool Triangle(ref NativeList<SupportPoint> simplex, ref float3 dir)
        {
            SupportPoint a = simplex[2];
            SupportPoint b = simplex[1];
            SupportPoint c = simplex[0];

            float3 ab = b.Point - a.Point;
            float3 ac = c.Point - a.Point;
            float3 ao = -a.Point;

            float3 abc = math.cross(ab, ac);

            float3 abPerp = math.cross(math.cross(ac, abc), ac);
            if (math.dot(abPerp, ao) > 0)
            {
                simplex.RemoveAt(0);
                dir = abPerp;
                return false;
            }

            float3 acPerp = math.cross(math.cross(abc, ab), ab);
            if (math.dot(acPerp, ao) > 0)
            {
                simplex.RemoveAt(1);
                dir = acPerp;
                return false;
            }

            // 原点在三角形上方
            if (math.dot(abc, ao) > 0)
                dir = abc;
            else
            {
                (simplex[0], simplex[1]) = (simplex[1], simplex[0]); // 翻转
                dir = -abc;
            }

            return false;
        }
        
        private static bool Tetrahedron(ref NativeList<SupportPoint> simplex, ref float3 dir)
        {
            SupportPoint a = simplex[3];
            SupportPoint b = simplex[2];
            SupportPoint c = simplex[1];
            SupportPoint d = simplex[0];

            float3 ao = -a.Point;

            float3 abc = math.cross(b.Point - a.Point, c.Point - a.Point);
            float3 acd = math.cross(c.Point - a.Point, d.Point - a.Point);
            float3 adb = math.cross(d.Point - a.Point, b.Point - a.Point);

            if (math.dot(abc, ao) > 0)
            {
                simplex.RemoveAt(0);
                dir = abc;
                return false;
            }

            if (math.dot(acd, ao) > 0)
            {
                simplex.RemoveAt(2);
                dir = acd;
                return false;
            }

            if (math.dot(adb, ao) > 0)
            {
                simplex.RemoveAt(1);
                dir = adb;
                return false;
            }

            return true; // 原点在四面体内部
        }

        private static void EPA_Solve(in NativeConvex convexA, in NativeConvex convexB,ref NativeList<SupportPoint> simplex, out EPAResult result)
        {
            const int MaxEpaIteration = 128;
            result = default;
            result.HasResult = false;
            
            var faces = new NativeList<EPAFace>(Allocator.Temp);
            if (simplex.Length < 4)
            {
                faces.Dispose();
                return;
            }
            BuildInitialPolytope(ref simplex, ref faces);

            for (int iter = 0; iter < MaxEpaIteration; ++iter)
            {
                // 找到距离原点最近的面
                int closestIndex = -1;
                float minDist = float.MaxValue;
                for (int i = 0; i < faces.Length; ++i)
                {
                    var f = faces[i];
                    if (!f.Alive) continue;
                    float d = math.dot(f.Normal, f.A.Point);
                    if (d < minDist)
                    {
                        minDist = d;
                        closestIndex = i;
                    }
                }

                if (closestIndex == -1) break;

                var closest = faces[closestIndex];

                // 找到最近面法线方向上的支撑点
                SupportPoint p = Support(convexA, convexB, closest.Normal);
                float pDist = math.dot(p.Point, closest.Normal);

                // 如果没有更近点
                if (pDist - minDist < Epsilon)
                {
                    // produce result: normal (A->B), depth, contact points
                    // ensure normal points from A to B
                    
                    float3 n = closest.Normal;


                    // 原点在最近面上的投影点（Minkowski差内的点）
                    float3 proj = n * minDist;

                    //  计算投影点在最近面（三角形）上的重心坐标（用于插值接触点)
                    float3 paA = closest.A.PointA;
                    float3 paB = closest.B.PointA;
                    float3 paC = closest.C.PointA;

                    float3 pbA = closest.A.PointB;
                    float3 pbB = closest.B.PointB;
                    float3 pbC = closest.C.PointB;

                    // compute barycentric in triangle of minkowski points
                    float3 aP = closest.A.Point;
                    float3 bP = closest.B.Point;
                    float3 cP = closest.C.Point;

                    // compute barycentric of proj relative to triangle aP,bP,cP on plane
                    float3 v0 = bP - aP;
                    float3 v1 = cP - aP;
                    float3 v2 = proj - aP;

                    float d00 = math.dot(v0, v0);
                    float d01 = math.dot(v0, v1);
                    float d11 = math.dot(v1, v1);
                    float d20 = math.dot(v2, v0);
                    float d21 = math.dot(v2, v1);
                    float denom = d00 * d11 - d01 * d01;
                    float v = 0f, w = 0f, u = 0f;
                    if (math.abs(denom) > Epsilon)
                    {
                        v = (d11 * d20 - d01 * d21) / denom;
                        w = (d00 * d21 - d01 * d20) / denom;
                        u = 1.0f - v - w;
                    }
                    else
                    {
                        // degenerate triangle -> fallback equal weights
                        u = v = w = 1.0f / 3.0f;
                    }
                    
                    // 插值得到两个凸体上的接触点
                    float3 contactA = u * paA + v * paB + w * paC;
                    float3 contactB = u * pbA + v * pbB + w * pbC;
                    
                    float3 centerAB = result.ContactB - result.ContactA;
                    if (math.dot(n, centerAB) < 0)
                        n = math.normalize(n);

                    // 穿透深度（沿法线方向的重叠距离）
                    float depth = math.dot(n, contactA - contactB);

                    result.HasResult = true;
                    result.Normal = n;
                    result.Depth = depth;
                    result.ContactA = contactA;
                    result.ContactB = contactB;

                    faces.Dispose();
                    return;
                }

                // 扩展多面体：移除可见面，用边界边与新支撑点创建新面
                ExpandPolytope(ref faces, p);
            }

            faces.Dispose();
            
        }
        
        private static void BuildInitialPolytope(ref NativeList<SupportPoint> simplex, ref NativeList<EPAFace> faces)
        {
            faces.Add(new EPAFace(simplex[0], simplex[1], simplex[2],simplex[3]));
            faces.Add(new EPAFace(simplex[0], simplex[3], simplex[1],simplex[2]));
            faces.Add(new EPAFace(simplex[0], simplex[2], simplex[3],simplex[1]));
            faces.Add(new EPAFace(simplex[1], simplex[3], simplex[2],simplex[0]));
        }
        
        private static void ExpandPolytope(ref NativeList<EPAFace> faces, SupportPoint p)
        {
            var edgeBuffer = new NativeList<EPAEdge>(Allocator.Temp);

            // mark faces visible from p and collect their boundary edges
            for (int i = 0; i < faces.Length; ++i)
            {
                if (!faces[i].Alive) continue;
                var f = faces[i];
                if (math.dot(f.Normal, p.Point - f.A.Point) > 0f)
                {
                    // face visible -> remove and record edges
                    faces[i] = new EPAFace { Alive = false, A = f.A, B = f.B, C = f.C, Normal = f.Normal, Distance = f.Distance };
                    // add edges (A,B), (B,C), (C,A)
                    AddEdge(edgeBuffer, f.A, f.B);
                    AddEdge(edgeBuffer, f.B, f.C);
                    AddEdge(edgeBuffer, f.C, f.A);
                }
            }

            // create new faces from boundary edges paired with p
            for (int i = 0; i < edgeBuffer.Length; ++i)
            {
                var e = edgeBuffer[i];
                faces.Add(new EPAFace(e.Start, e.End, p));
            }

            edgeBuffer.Dispose();

            // Optionally: compact faces list (remove Alive=false entries) to keep sizes small
            for (int i = faces.Length - 1; i >= 0; --i)
                if (!faces[i].Alive) faces.RemoveAtSwapBack(i);
        }
        
        private static void AddEdge(NativeList<EPAEdge> edges, SupportPoint a, SupportPoint b)
        {
            // if reverse exists, remove it; otherwise add.
            for (int i = 0; i < edges.Length; ++i)
            {
                var e = edges[i];
                if (PointsEqual(e.Start.Point, b.Point) && PointsEqual(e.End.Point, a.Point))
                {
                    // remove twin
                    edges.RemoveAtSwapBack(i);
                    return;
                }
            }
            edges.Add(new EPAEdge { Start = a, End = b });
        }
        
        private static bool PointsEqual(float3 p, float3 q)
        {
            return math.lengthsq(p - q) <= Epsilon * Epsilon;
        }
        
        private static SupportPoint Support(in NativeConvex A, in NativeConvex B, float3 dir)
        {
            float3 pa = A.Hull.GetSupportWorld(A.Transform,dir,A.Scale);
            float3 pb = B.Hull.GetSupportWorld(B.Transform,-dir,B.Scale);
            return new SupportPoint(pa,pb);
        }
        
        public static float3 ClosestPointOnTriangleToOrigin(float3 a, float3 b, float3 c, out float u, out float v, out float w)
        {
            float3 ab = b - a;
            float3 ac = c - a;
            float3 ap = -a;

            float d1 = math.dot(ab, ap);
            float d2 = math.dot(ac, ap);

            // 顶点 A 区域
            if (d1 <= 0f && d2 <= 0f)
            {
                u = 1f; v = 0f; w = 0f;
                return a;
            }

            float3 bp = -b;
            float d3 = math.dot(ab, bp);
            float d4 = math.dot(ac, bp);

            // 顶点 B 区域
            if (d3 >= 0f && d4 <= d3)
            {
                u = 0f; v = 1f; w = 0f;
                return b;
            }

            // 边 AB 区域
            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0f && d1 >= 0f && d3 <= 0f)
            {
                float t = d1 / (d1 - d3);
                u = 1f - t; v = t; w = 0f;
                return a + t * ab;
            }

            float3 cp = -c;
            float d5 = math.dot(ab, cp);
            float d6 = math.dot(ac, cp);

            // 顶点 C 区域
            if (d6 >= 0f && d5 <= d6)
            {
                u = 0f; v = 0f; w = 1f;
                return c;
            }

            // 边 AC 区域
            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0f && d2 >= 0f && d6 <= 0f)
            {
                float t = d2 / (d2 - d6);
                u = 1f - t; v = 0f; w = t;
                return a + t * ac;
            }

            // 边 BC 区域
            float va = d3 * d6 - d5 * d4;
            if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
            {
                float t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                u = 0f; v = 1f - t; w = t;
                return b + t * (c - b);
            }

            // 面内部区域
            float denom = 1f / (va + vb + vc);
            v = vb * denom;
            w = vc * denom;
            u = 1f - v - w;
            return u * a + v * b + w * c;
        }
    }
    [BurstCompile]
    public struct SupportPoint
    {
        public float3 Point; // 闵可夫斯基差 = pA - pB
        public float3 PointA; // support point in A
        public float3 PointB; // support point in B

        public SupportPoint(float3 pointA, float3 pointB)
        {
            PointA = pointA;
            PointB = pointB;
            Point = pointA - pointB;
        }
    }
    [BurstCompile]
    public struct EPAResult
    {
        public bool HasResult;
        public float3 Normal;   // unit, from A to B
        public float Depth;     // >0 means penetration
        public float3 ContactA; // point on A
        public float3 ContactB; // point on B
        public int BodyA;
        public int BodyB;
    }
    

    public struct EPAFace
    {
        public SupportPoint A, B, C;
        public float3 Normal;
        public float Distance;
        public bool Alive;

        public EPAFace(SupportPoint a, SupportPoint b, SupportPoint c,SupportPoint d)
        {
            A = a; B = b; C = c;
            float3 n = math.cross(b.Point - a.Point, c.Point - a.Point);
            if (math.dot(n, d.Point - a.Point) > 0) n = -n;
            float len = math.length(n);
            if (len > Single.Epsilon) n = n / len;
            else n = new float3(0, 1, 0);
            Normal = n;
            Distance = math.dot(Normal, a.Point);
            Alive = true;
        }
        public EPAFace(SupportPoint a, SupportPoint b, SupportPoint c)
        {
            A = a; B = b; C = c;
            float3 n = math.cross(b.Point - a.Point, c.Point - a.Point);
            float len = math.length(n);
            if (len > Single.Epsilon) n = n / len;
            else n = new float3(0, 1, 0);
            Normal = n;
            Distance = math.dot(Normal, a.Point);
            Alive = true;
        }
    }

    public struct EPAEdge
    {
        public SupportPoint Start;
        public SupportPoint End;
    }
}