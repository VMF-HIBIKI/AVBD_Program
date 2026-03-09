using Collision;
using Unity.Mathematics;
using UnityEngine;

namespace Utils
{
    public static class CollisionDebugDizmos
    {
        const float PointRadius = 0.02f;
        const float NormalLength = 0.5f;
        const float ArrowHeadSize = 0.06f;

        public static void Draw(EPAResult result)
        {
            // EPA contact points
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(result.ContactA, PointRadius);
            Gizmos.DrawSphere(result.ContactB, PointRadius);
            Gizmos.DrawLine(result.ContactA, result.ContactB);

            // Normal arrow from midpoint
            float3 mid = (result.ContactA + result.ContactB) * 0.5f;
            float3 tip = mid + result.Normal * NormalLength;
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(mid, tip);
            DrawArrowHead(mid, tip);
        }

        public static void DrawManifoldPoint(float3 contactA, float3 contactB,
                                              float3 normal, float depth)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(contactA, PointRadius * 0.8f);
            Gizmos.DrawSphere(contactB, PointRadius * 0.8f);
            Gizmos.DrawLine(contactA, contactB);

            float3 mid = (contactA + contactB) * 0.5f;
            float3 tip = mid + normal * NormalLength * 0.6f;
            Gizmos.color = new Color(0f, 1f, 0.5f);
            Gizmos.DrawLine(mid, tip);
            DrawArrowHead(mid, tip);
        }

        static void DrawArrowHead(float3 from, float3 to)
        {
            float3 dir = math.normalize(to - from);
            float3 right = math.abs(dir.y) < 0.99f
                ? math.normalize(math.cross(dir, math.up()))
                : math.normalize(math.cross(dir, math.right()));
            float3 up = math.normalize(math.cross(right, dir));

            float3 back = to - dir * ArrowHeadSize;
            Gizmos.DrawLine(to, back + right * ArrowHeadSize * 0.4f);
            Gizmos.DrawLine(to, back - right * ArrowHeadSize * 0.4f);
            Gizmos.DrawLine(to, back + up * ArrowHeadSize * 0.4f);
            Gizmos.DrawLine(to, back - up * ArrowHeadSize * 0.4f);
        }
    }
}
