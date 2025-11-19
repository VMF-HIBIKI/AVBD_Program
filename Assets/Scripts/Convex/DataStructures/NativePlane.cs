using Unity.Mathematics;
using static Unity.Mathematics.math;

namespace Convex.DataStructures
{
    public struct NativePlane
    {
        //平面方程可以表示为Normal · P = Offset
        //normal是法向量
        //P是平面一点
        //Offset是从原点沿着法线方向到平面的距离
        
        public float3 Normal;
        
        public float Offset;

        public int FirstHalfedge;
        
        public NativePlane(float3 normal, float offset , int firstHalfedge)
        {
            Normal = normal;
            Offset = offset;
            FirstHalfedge = firstHalfedge;
        }

        /// <summary>
        /// 计算某点到这个平面的距离
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public float Distance(float3 point)
        {
            return dot(Normal, point) - Offset;
        }
        
        /// <summary>
        /// 计算一个点到这个平面的最近点
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public float3 ClosestPoint(float3 point)
        {
            return point - Distance(point) * normalize(Normal);
        }
        
        public static NativePlane operator *(RigidTransform tran, NativePlane plane)
        {
            float3 normal = mul(tran.rot, plane.Normal);
            return new NativePlane(normal, plane.Offset + dot(tran.pos, normal),plane.FirstHalfedge);
        }
        
        
    }
}
