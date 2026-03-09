using UnityEngine;
using Unity.Mathematics;

public static class MortonDebugGizmos
{
    static readonly Color[] DepthColors =
    {
        Color.white,
        Color.cyan,
        Color.green,
        Color.yellow,
        new Color(1f, 0.5f, 0f),
        Color.blue,
        Color.magenta,
        Color.red
    };

    public static void Draw(ProcessSystem system)
    {
        var nodes = system.LBVHNodesRead;
        int leafCount = system.LBVHLeafCount;
        if (!nodes.IsCreated || leafCount < 2) return;

        int ic = leafCount - 1;
        DrawNode(nodes, 0, ic, 0);

        // Collision pair lines
        Gizmos.color = Color.red;
        var pairs = system.CollisionPairsRead;
        var aabbs = system.AABBsRead;
        for (int i = 0; i < pairs.Length; i++)
        {
            var p = pairs[i];
            Gizmos.DrawLine(aabbs[p.x].Center, aabbs[p.y].Center);
        }
    }

    static void DrawNode(Unity.Collections.NativeArray<LBVHNode> nodes,
                          int idx, int ic, int depth)
    {
        var nd = nodes[idx];
        bool isLeaf = nd.objectIdx >= 0;
        var col = DepthColors[depth % DepthColors.Length];

        if (isLeaf)
        {
            Gizmos.color = col;
            Gizmos.DrawWireCube(nd.bounds.Center, nd.bounds.Size);
        }
        else
        {
            // Internal: draw bounds semi-transparent
            col.a = 0.3f;
            Gizmos.color = col;
            Gizmos.DrawWireCube(nd.bounds.Center, nd.bounds.Size);

            // Lines from this node's center to children's centers
            float3 c = nd.bounds.Center;
            Gizmos.color = new Color(col.r, col.g, col.b, 0.6f);
            Gizmos.DrawLine(c, nodes[nd.left].bounds.Center);
            Gizmos.DrawLine(c, nodes[nd.right].bounds.Center);

            DrawNode(nodes, nd.left,  ic, depth + 1);
            DrawNode(nodes, nd.right, ic, depth + 1);
        }
    }
}
