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
                    
                    /*
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
                    float depth = math.dot(n, contactA - contactB);*/

                    result.HasResult = true;
                    result.Normal = n;
                    result.Depth = pDist;

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
        
        // 获取特征面的顶点列表（世界坐标）和面的信息
        private static void GetFeatureFace(
            in NativeConvex convex, 
            float3 searchDir, // 指向凸包外部的法线方向
            ref NativeList<float3> faceVertices, 
            out float3 faceNormal, 
            out float3 faceCenter)
        {
            faceVertices.Clear();
            faceNormal = float3.zero;
            faceCenter = float3.zero;

            // 1. 遍历所有面，找到法线与 searchDir 点积最大的面
            int bestPlaneIndex = -1;
            float maxDot = -float.MaxValue;

            // 缓存旋转矩阵，用于将局部法线转世界
            float3x3 rotMatrix = new float3x3(convex.Transform.rot);

            for (int i = 0; i < convex.Hull.PlaneCount; i++)
            {
                NativePlane plane = convex.Hull.Planes[i];
                float3 worldNormal = math.mul(rotMatrix, plane.Normal);
                float dot = math.dot(worldNormal, searchDir);

                if (dot > maxDot)
                {
                    maxDot = dot;
                    bestPlaneIndex = i;
                    faceNormal = worldNormal;
                }
            }

            if (bestPlaneIndex == -1) return;

            // 2. 提取该面的所有顶点
            NativePlane bestPlane = convex.Hull.Planes[bestPlaneIndex];
            int startHeIndex = bestPlane.FirstHalfedge;
            int currHeIndex = startHeIndex;
    
            // 遍历半边循环
            do
            {
                NativeHalfEdge he = convex.Hull.Edges[currHeIndex];
                NativeVertex v = convex.Hull.Vertices[he.StartVertex];

                // 局部转世界： Scale -> Rotate -> Translate
                float3 localPos = v.Position * convex.Scale;
                float3 worldPos = math.transform(convex.Transform, localPos);

                faceVertices.Add(worldPos);
                faceCenter += worldPos;

                currHeIndex = he.NextHalfedge;
            } 
            while (currHeIndex != startHeIndex);

            faceCenter /= faceVertices.Length;
        }
        
        // 用一个平面裁剪多边形，保留平面“内侧”的点
        private static void ClipByPlane(
            float3 planeNormal, 
            float3 planePos, 
            NativeList<float3> inputPolygon, 
            NativeList<float3> outputPolygon)
        {
            if (inputPolygon.Length == 0) return;

            outputPolygon.Clear();
    
            float3 vPrev = inputPolygon[inputPolygon.Length - 1];
            float dPrev = math.dot(planeNormal, vPrev - planePos);

            for (int i = 0; i < inputPolygon.Length; i++)
            {
                float3 vCurr = inputPolygon[i];
                float dCurr = math.dot(planeNormal, vCurr - planePos);

                // 判断两点相对于平面的位置
                // d > 0: 在平面外（被裁剪掉的方向，视具体法线定义而定，这里假设法线指向内部为正）
                // 实际上 Sutherland-Hodgman 通常定义法线指向保留的一侧
        
                // Case 1: Prev 在内，Curr 在内 -> 保留 Curr
                if (dPrev >= 0 && dCurr >= 0)
                {
                    outputPolygon.Add(vCurr);
                }
                // Case 2: Prev 在内，Curr 在外 -> 计算交点，保留交点
                else if (dPrev >= 0 && dCurr < 0)
                {
                    float3 intersection = LinePlaneIntersection(vPrev, vCurr, planePos, planeNormal);
                    outputPolygon.Add(intersection);
                }
                // Case 3: Prev 在外，Curr 在内 -> 计算交点，保留交点和 Curr
                else if (dPrev < 0 && dCurr >= 0)
                {
                    float3 intersection = LinePlaneIntersection(vPrev, vCurr, planePos, planeNormal);
                    outputPolygon.Add(intersection);
                    outputPolygon.Add(vCurr);
                }
                // Case 4: 都在外 -> 都不保留

                vPrev = vCurr;
                dPrev = dCurr;
            }
        }

        private static float3 LinePlaneIntersection(float3 p1, float3 p2, float3 planePos, float3 planeNormal)
        {
            float3 u = p2 - p1;
            float dot = math.dot(planeNormal, u);
            if (math.abs(dot) < 1e-6f) return p1; // 平行，防除零
            float t = math.dot(planeNormal, planePos - p1) / dot;
            return p1 + u * t;
        }
        
        public static void BuildManifold(
            NativeConvex convexA, 
            NativeConvex convexB, 
            float3 epaNormal, // 必须是从 A 指向 B 的法线
            ref NativeList<ManifoldPoint> manifolds)
        {
            manifolds.Clear();

            // 临时内存分配
            var polyA = new NativeList<float3>(8, Allocator.Temp);
            var polyB = new NativeList<float3>(8, Allocator.Temp);

            // 1. 寻找特征面
            // A 的特征面：寻找法线与 epaNormal 最一致的面
            GetFeatureFace(convexA, epaNormal, ref polyA, out float3 normA, out float3 centerA);
            
            // B 的特征面：寻找法线与 -epaNormal 最一致的面
            GetFeatureFace(convexB, -epaNormal, ref polyB, out float3 normB, out float3 centerB);

            // 2. 确定参考面 (Reference) 和 入射面 (Incident)
            // 通常选择与碰撞法线“平行度”更差的那个面作为入射面（Incident），因为它更像是在戳另一个面
            // 这里简化判断：看法线投影长度
            
            bool flip = false;
            NativeList<float3> refPoly; // 参考面多边形
            NativeList<float3> incPoly; // 入射面多边形
            float3 refNormal;
            float3 refCenter;

            // 这里的逻辑是：谁的法线更平行于碰撞轴，谁就是参考面
            if (math.abs(math.dot(normA, epaNormal)) > math.abs(math.dot(normB, epaNormal)))
            {
                // A 是参考面
                refPoly = polyA;
                incPoly = polyB;
                refNormal = normA;
                refCenter = centerA;
                flip = false; 
            }
            else
            {
                // B 是参考面
                refPoly = polyB;
                incPoly = polyA;
                refNormal = normB;
                refCenter = centerB;
                flip = true;
            }

            // 3. 准备裁剪
            // 我们将把入射面(Inc) 裁剪到 参考面(Ref) 的棱柱体内
            
            // 复制一份入射面作为初始裁剪多边形
            var clippedPoly = new NativeList<float3>(incPoly.Length + 4, Allocator.Temp);
            for(int i=0; i<incPoly.Length; i++) clippedPoly.Add(incPoly[i]);
            
            var tempPoly = new NativeList<float3>(clippedPoly.Length, Allocator.Temp);

            // 4. 对参考面的每一条边，构建侧平面进行裁剪
            // 参考面的顶点顺序应该是逆时针或顺时针，这决定了侧平面的法线方向
            // 假设 GetFeatureFace 拿出的顶点是逆时针的，面法线向外
            
            float3 vPrev = refPoly[refPoly.Length - 1];
            for (int i = 0; i < refPoly.Length; i++)
            {
                float3 vCurr = refPoly[i];
                float3 edge = vCurr - vPrev;
                
                // 计算指向参考面内部的侧平面法线
                // 侧平面法线 = Cross(RefNormal, Edge).Normalize()
                // 注意：这里需要确认具体的叉乘顺序以指向内部
                float3 sidePlaneNormal = math.normalize(math.cross(refNormal, edge));

                // 执行裁剪
                ClipByPlane(sidePlaneNormal, vCurr, clippedPoly, tempPoly);
                
                // 交换 Buffer 准备下一轮
                clippedPoly.Clear();
                for(int k=0; k<tempPoly.Length; k++) clippedPoly.Add(tempPoly[k]);
                tempPoly.Clear();

                vPrev = vCurr;
            }

            // 5. 最后一步：只保留在参考面“下方”的点（真正的穿透点）
            // 参考面方程： Dot(n, p) - d = 0
            // 深度 = Dot(n, p - center)
            // 这里的 refNormal 是指向外部的
            
            // 我们需要保留在参考面“里面”的点。如果 Ref 是 A，Normal A 向外。Inc 是 B，B 的点应该在 A 内部。
            // 所以 Dot(refNormal, point - refCenter) 应该 < 0
            
            for (int i = 0; i < clippedPoly.Length; i++)
            {
                float3 p = clippedPoly[i];
                float dist = math.dot(refNormal, p - refCenter);

                // 如果距离为负，说明点在参考面下方（发生穿透）
                // 设置一个小容差 (Positive Tolerance) 允许轻微浮点误差
                if (dist <= 0.01f) 
                {
                    // 投影点作为参考面上的点
                    float3 projectedPoint = p - refNormal * dist;
                    
                    ManifoldPoint mp = new ManifoldPoint();
                    mp.Normal = epaNormal;
                    mp.Depth = -dist; // 深度通常取正值

                    if (!flip)
                    {
                        // Ref 是 A，Inc 是 B
                        // 裁剪后的点 p 来自 B (Incident)
                        mp.ContactB = p;
                        mp.ContactA = projectedPoint; // A 上的对应点
                    }
                    else
                    {
                        // Ref 是 B，Inc 是 A
                        // 裁剪后的点 p 来自 A (Incident)
                        mp.ContactA = p;
                        mp.ContactB = projectedPoint; // B 上的对应点
                    }
                    
                    manifolds.Add(mp);
                }
            }

            // 清理
            polyA.Dispose();
            polyB.Dispose();
            clippedPoly.Dispose();
            tempPoly.Dispose();
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
    
    [BurstCompile]
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
    [BurstCompile]
    public struct EPAEdge
    {
        public SupportPoint Start;
        public SupportPoint End;
    }
}