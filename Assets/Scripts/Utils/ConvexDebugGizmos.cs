using UnityEngine;

namespace Utils
{
    public static class ConvexDebugGizmos
    {
        private static float vertexGizmoRadius = 0.05f;
        public static void Draw(NativeConvex inConvex,Transform transform)
        {
            var hull = inConvex.Hull;
            Gizmos.color = Color.blue;
            foreach (var vertex in hull.Vertices)
            {
                // 将局部坐标转换为世界坐标
                Vector3 worldPos = transform.TransformPoint((Vector3)vertex.Position);
                Gizmos.DrawSphere(worldPos, vertexGizmoRadius);
            }
            
            Gizmos.color = Color.green;
            foreach (var edge in hull.Edges)
            {
                // 获取当前半边的起点
                var startVertex = hull.Vertices[edge.StartVertex];
                Vector3 startWorldPos = transform.TransformPoint((Vector3)startVertex.Position);
                    
                // 获取下一条半边的起点（当前半边的终点）
                var nextEdge = hull.Edges[edge.NextHalfedge];
                var endVertex = hull.Vertices[nextEdge.StartVertex];
                Vector3 endWorldPos = transform.TransformPoint((Vector3)endVertex.Position);
                    
                // 绘制半边线条
                Gizmos.DrawLine(startWorldPos, endWorldPos);
            }
        }
    }
}