using Unity.Mathematics;

public struct ManifoldPoint
{
    public float3 ContactA; // A 上的点（参考面上的投影或原始点）
    public float3 ContactB; // B 上的点（被裁剪后的点）
    public float Depth;     // 穿透深度
    public float3 Normal;   // 法线 (A -> B)
}