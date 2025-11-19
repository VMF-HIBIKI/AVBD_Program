using UnityEngine;
using Unity.Mathematics;

public static class MortonDebugGizmos
{
    // 八叉树层级颜色
    private static Color[] LevelColors = new Color[]
    {
        Color.white,
        Color.cyan,
        Color.green,
        Color.yellow,
        new Color(1f,0.5f,0f),
        Color.blue,
        Color.magenta
    };

    public static void Draw(ProcessSystem system)
    {
        if (!system.AABBsRead.IsCreated) return;

        // 绘制八叉树逻辑节点
        DrawOctree(system.SceneBounds, system.MaxDepth, 0);

        // 绘制物体AABB
        for (int i = 0; i < system.ObjectCount; i++)
        {
            var aabb = system.AABBsRead[i];
            int level = system.LevelsRead[i];
            Gizmos.color = LevelColors[level % LevelColors.Length];
            Gizmos.DrawWireCube(aabb.Center, aabb.Size);
        }

        // 绘制可能碰撞对
        Gizmos.color = Color.red;
        for (int i = 0; i < system.CollisionPairsRead.Length; i++)
        {
            var p = system.CollisionPairsRead[i];
            // 只绘制一条线，连接两个物体中心
            var a = system.AABBsRead[p.x].Center;
            var b = system.AABBsRead[p.y].Center;
            Gizmos.DrawLine(a, b);
        }
    }

    // 递归绘制八叉树节点
    private static void DrawOctree(AABB node, int maxDepth, int currentDepth)
    {
        if (currentDepth > maxDepth) return;

        // 绘制当前节点边界
        Gizmos.color = Color.gray;
        Gizmos.DrawWireCube(node.Center, node.Size);

        // 子节点尺寸 = 父节点一半
        float3 childSize = node.Size * 0.5f;
        float3 c = node.Center;
        Gizmos.DrawSphere(c,0.2f);
        // 遍历8个子节点
        for (int xi = -1; xi <= 1; xi += 2)
        {
            for (int yi = -1; yi <= 1; yi += 2)
            {
                for (int zi = -1; zi <= 1; zi += 2)
                {
                    float3 offset = new float3(xi, yi, zi) * childSize * 0.5f;
                    var child = new AABB
                    {
                        Min = c + offset - childSize * 0.5f,
                        Max = c + offset + childSize * 0.5f
                    };
                    DrawOctree(child, maxDepth, currentDepth + 1);
                }
            }
        }
    }
}
