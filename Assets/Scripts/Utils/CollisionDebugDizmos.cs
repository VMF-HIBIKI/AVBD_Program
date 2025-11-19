using Collision;
using UnityEngine;

namespace Utils
{
    public static class CollisionDebugDizmos
    {
        const float CollisionPointRadius = 0.01f;
        public static void Draw(EPAResult result)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(result.ContactA, CollisionPointRadius);
            Gizmos.DrawSphere(result.ContactB, CollisionPointRadius);
            Gizmos.DrawLine(result.ContactA, result.ContactB);
        }
    }
}