using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using AVBD;
using Collision;
using Convex;
using Unity.VisualScripting;
using Utils;

public class ProcessSystem : MonoBehaviour
{
    [Header("Settings")]
    [Range(2,20)]
    public int MaxDepth = 5;
    public bool DrawBVH = true;
    public bool DrawConvex = true;
    public bool DrawCollision = true;

    [SerializeField] [Range(0f,1f)] private float alpha;
    [SerializeField] [Range(0f,0.99f)] private float gamma;
    [SerializeField] [Range(0.01f,0.99f)] private float friction;
    [SerializeField] [Range(10f, 100000f)] private float beta;
    public float Gravity = 9.8f;
    [Range(1,150)] public int ConstraintSolverIteratorCount = 10;

    private List<ulong> Granularitys = new List<ulong>();
    private List<ulong> BitMasks = new List<ulong>();

    private AABB _sceneBounds;
    private static List<DetectionBody> Bodies = new List<DetectionBody>();
    private static Dictionary<Tuple<int,int>,Tuple<NativeConvex,NativeConvex>> ConvexesDic = new Dictionary<Tuple<int, int>, Tuple<NativeConvex, NativeConvex>>();

    private NativeArray<AABB> AABBs;
    private NativeArray<float3> Positions;
    private NativeArray<quaternion> Rotations;
    private NativeArray<float3> Scales;
    private NativeArray<Bounds> LocalBounds;
    private NativeArray<ulong> MortonCodes;
    private NativeArray<int> Levels;
    private NativeArray<int> ObjectIndices;
    private NativeList<int2> CollisionPairs;
    private NativeList<EPAResult> EpaResults;
    private NativeArray<LBVHNode> _lbvhNodesDebug;
    private int _lbvhLeafCount;

    private AvbdSolver solver;

    private struct DebugContact
    {
        public float3 pointA, pointB, normal;
        public float depth;
    }
    private readonly List<DebugContact> _debugContacts = new List<DebugContact>();

    public AABB SceneBounds => _sceneBounds;
    public NativeList<int2> CollisionPairsRead => CollisionPairs;
    public NativeArray<AABB> AABBsRead => AABBs;
    public NativeArray<int> LevelsRead => Levels;
    public NativeArray<LBVHNode> LBVHNodesRead => _lbvhNodesDebug;
    public int LBVHLeafCount => _lbvhLeafCount;
    public int ObjectCount => Bodies.Count;

    private int capacity = 1024;

    void Awake()
    {
        InitializeBroadPhaseContext();
        solver = new AvbdSolver();
    }

    private void Start()
    {
        // no longer needed: prevVelocity is zero-initialized in DetectionBody
    }

    private void InitializeBroadPhaseContext()
    {
        InitializeSceneBounds();
        Allocate(capacity);
        InitializeMortonCodeNeeds();
    }

    #region BroadPhaseContext
    public void InitializeMortonCodeNeeds()
    {
        for (int i = 0; i <= 20; i++)
        {
            Granularitys.Add((1UL << i) - 1UL);
            BitMasks.Add((1UL << i) - 1UL);
        }
    }

    public void InitializeSceneBounds()
    {
        if (Bodies.Count == 0)
        {
            _sceneBounds = new AABB
            {
                Min = new float3(-0.5f, -0.5f, -0.5f),
                Max = new float3(0.5f, 0.5f, 0.5f)
            };
            return;
        }

        float3 min = new float3(float.MaxValue);
        float3 max = new float3(float.MinValue);

        foreach (var body in Bodies)
        {
            if (body == null || body.MeshFilter == null) continue;
            var mesh = body.MeshFilter.sharedMesh;
            if (mesh == null) continue;
            var b = mesh.bounds;
            float3 center  = (float3)b.center;
            float3 extents = (float3)b.extents;
            float3 scale   = (float3)body.transform.lossyScale;
            quaternion rot  = body.transform.rotation;
            float3 pos     = (float3)body.transform.position;

            float3 sc = center * scale;
            float3 se = extents * math.abs(scale);
            float3x3 rm = new float3x3(rot);
            float3 we = math.abs(rm.c0) * se.x
                      + math.abs(rm.c1) * se.y
                      + math.abs(rm.c2) * se.z;
            float3 wc = math.rotate(rot, sc) + pos;
            min = math.min(min, wc - we);
            max = math.max(max, wc + we);
        }

        float3 c = (min + max) * 0.5f;
        float maxExtent = math.cmax(max - min) * 0.5f;

        _sceneBounds = new AABB
        {
            Min = c - new float3(maxExtent + 0.1f),
            Max = c + new float3(maxExtent + 0.1f)
        };
    }

    void Allocate(int cap)
    {
        Dispose();

        AABBs = new NativeArray<AABB>(cap, Allocator.Persistent);
        Positions = new NativeArray<float3>(cap, Allocator.Persistent);
        Rotations = new NativeArray<quaternion>(cap, Allocator.Persistent);
        Scales = new NativeArray<float3>(cap, Allocator.Persistent);
        LocalBounds = new NativeArray<Bounds>(cap, Allocator.Persistent);
        MortonCodes = new NativeArray<ulong>(cap, Allocator.Persistent);
        Levels = new NativeArray<int>(cap, Allocator.Persistent);
        ObjectIndices = new NativeArray<int>(cap, Allocator.Persistent);
        CollisionPairs = new NativeList<int2>(cap * 2, Allocator.Persistent);
    }
    #endregion

    public void RefreshSceneBounds()
    {
        InitializeSceneBounds();
    }

    public static void Register(DetectionBody body)
    {
        if (!Bodies.Contains(body))
            Bodies.Add(body);
    }

    public static void Unregister(DetectionBody body)
    {
        if (Bodies.Contains(body))
            Bodies.Remove(body);
    }

    public static DetectionBody GetBodyById(int id)
    {
        if (id < 0 || id >= Bodies.Count)
            return null;
        return Bodies[id];
    }

    private void Update()
    {
        if (EpaResults.IsCreated) EpaResults.Clear();
        UpdateBroadPhaseProcess();
        CreatConvexesByPairs();
        NarrowPhaseProcess();
        UpdateBodies();
    }

    private void UpdateBroadPhaseProcess()
    {
        if (Bodies.Count == 0) return;
        if (Bodies.Count > AABBs.Length) Allocate(Bodies.Count * 2);

        RefreshSceneBounds();

        for (int i = 0; i < Bodies.Count; i++)
        {
            var t = Bodies[i].transform;
            Positions[i] = t.position;
            Rotations[i] = t.rotation;
            Scales[i] = t.lossyScale;
            LocalBounds[i] = Bodies[i].MeshFilter.sharedMesh.bounds;
            ObjectIndices[i] = i;
        }

        var meshJob = new MeshToAABBJob
        {
            Positions = Positions,
            Rotations = Rotations,
            Scales = Scales,
            LocalBounds = LocalBounds,
            WorldAABBs = AABBs
        }.Schedule(Bodies.Count, 64);

        var mortonJob = new MortonCodeJob
        {
            AABBs = AABBs,
            SceneBounds = SceneBounds,
            MaxDepth = MaxDepth,
            MortonCodes = MortonCodes,
            Levels = Levels,
            Granularity = Granularitys[MaxDepth],
            BitMask = BitMasks[MaxDepth],
        }.Schedule(Bodies.Count, 64, meshJob);

        var sortJob = new SortJob
        {
            MortonCodes = MortonCodes,
            ObjectIndices = ObjectIndices,
            Count = Bodies.Count
        }.Schedule(mortonJob);

        sortJob.Complete();

        int n = Bodies.Count;
        if (n < 2) return;

        int ic = n - 1;
        int nodeCount = 2 * n - 1;

        var lbvhNodes = new NativeArray<LBVHNode>(nodeCount, Allocator.TempJob);
        var counters  = new NativeArray<int>(ic, Allocator.TempJob, NativeArrayOptions.ClearMemory);

        var initJob = new InitLBVHJob
        {
            aabbs = AABBs,
            sortedIndices = ObjectIndices,
            internalCount = ic,
            nodes = lbvhNodes
        }.Schedule(nodeCount, 64);

        var buildJob = new BuildRadixTreeJob
        {
            sortedCodes = MortonCodes,
            leafCount = n,
            nodes = lbvhNodes
        }.Schedule(ic, 64, initJob);

        var boundsJob = new ComputeBVHBoundsJob
        {
            leafCount = n,
            nodes = lbvhNodes,
            counters = counters
        }.Schedule(n, 64, buildJob);

        CollisionPairs.Clear();
        int maxPairs = math.max(n * 16, 256);
        if (CollisionPairs.Capacity < maxPairs)
            CollisionPairs.SetCapacity(maxPairs);

        var traversalJob = new LBVHTraversalJob
        {
            nodes = lbvhNodes,
            leafCount = n,
            pairs = CollisionPairs.AsParallelWriter()
        }.Schedule(n, 1, boundsJob);

        traversalJob.Complete();

        if (_lbvhNodesDebug.IsCreated) _lbvhNodesDebug.Dispose();
        _lbvhNodesDebug = new NativeArray<LBVHNode>(nodeCount, Allocator.Persistent);
        NativeArray<LBVHNode>.Copy(lbvhNodes, _lbvhNodesDebug);
        _lbvhLeafCount = n;

        lbvhNodes.Dispose();
        counters.Dispose();
    }

    private void CreatConvexesByPairs()
    {
        if (CollisionPairs.IsEmpty)
        {
            foreach (var convexes in ConvexesDic.Values)
            {
                if (convexes.Item1.IsCreated()) convexes.Item1.Dispose();
                if (convexes.Item2.IsCreated()) convexes.Item2.Dispose();
            }
            ConvexesDic.Clear();
            return;
        }

        foreach (var convexes in ConvexesDic.Values)
        {
            if (convexes.Item1.IsCreated()) convexes.Item1.Dispose();
            if (convexes.Item2.IsCreated()) convexes.Item2.Dispose();
        }
        ConvexesDic.Clear();

        foreach (var Pair in CollisionPairs)
        {
            var pairA = ConvexConstructor.CreatConvex(Bodies[Pair.x]);
            var pairB = ConvexConstructor.CreatConvex(Bodies[Pair.y]);
            Tuple<int, int> key = new Tuple<int, int>(Pair.x, Pair.y);
            Tuple<int, int> inverseKey = new Tuple<int, int>(Pair.y, Pair.x);
            if (ConvexesDic.ContainsKey(key) || ConvexesDic.ContainsKey(inverseKey)) continue;
            ConvexesDic[key] = new Tuple<NativeConvex, NativeConvex>(pairA, pairB);
        }
    }

    private void NarrowPhaseProcess()
    {
        if (EpaResults.IsCreated) EpaResults.Dispose();
        EpaResults = new NativeList<EPAResult>(ConvexesDic.Count, Allocator.Persistent);

        solver.ClearContacts();
        _debugContacts.Clear();

        foreach (var Pair in ConvexesDic)
        {
            var convexPair = Pair.Value;
            if (!GJK_EPA.DetectedCollisionAndResolve(convexPair.Item1, convexPair.Item2, out var result1))
            {
                result1 = default;
                EpaResults.Add(result1);
                continue;
            }

            result1.BodyA = Pair.Key.Item1;
            result1.BodyB = Pair.Key.Item2;
            EpaResults.Add(result1);

            var manifolds = new NativeList<ManifoldPoint>(4, Allocator.Temp);
            GJK_EPA.BuildManifold(convexPair.Item1, convexPair.Item2, result1.Normal, ref manifolds);

            if (manifolds.Length > 0)
            {
                foreach (var pt in manifolds)
                {
                    AddContactToSolver(result1.BodyA, result1.BodyB,
                                       result1.Normal, pt.ContactA, pt.ContactB,
                                       pt.Depth);
                    _debugContacts.Add(new DebugContact
                    {
                        pointA = pt.ContactA, pointB = pt.ContactB,
                        normal = result1.Normal, depth = pt.Depth
                    });
                }
            }
            else
            {
                AddContactToSolver(result1.BodyA, result1.BodyB,
                                   result1.Normal, result1.ContactA, result1.ContactB,
                                   result1.Depth);
                _debugContacts.Add(new DebugContact
                {
                    pointA = result1.ContactA, pointB = result1.ContactB,
                    normal = result1.Normal, depth = result1.Depth
                });
            }

            manifolds.Dispose();
        }
    }

    void AddContactToSolver(int bodyA, int bodyB, float3 epaNormal,
                            float3 contactA, float3 contactB, float depth)
    {
        // EPA gives normal "from A to B"; solver convention is "from B to A".
        float3 solverNormal = -epaNormal;

        float fric = math.min(Bodies[bodyA].friction, Bodies[bodyB].friction);

        // Skip if both static
        if (Bodies[bodyA].isStatic && Bodies[bodyB].isStatic) return;

        // Ensure bodyA is dynamic; swap if needed so solver always drives bodyA.
        int sA = bodyA, sB = bodyB;
        float3 cA = contactA, cB = contactB;
        float3 n = solverNormal;

        if (Bodies[sA].isStatic)
        {
            sA = bodyB; sB = bodyA;
            cA = contactB; cB = contactA;
            n = -solverNormal;
        }

        solver.AddContact(sA, sB, n, cA, cB, depth, fric);
    }

    private void UpdateBodies()
    {
        solver.gravity = new float3(0, -Gravity, 0);
        solver.iterations = ConstraintSolverIteratorCount;
        solver.alpha = alpha;
        solver.beta = beta;
        solver.gamma = gamma;
        solver.SetBodies(Bodies);
        solver.Step(Time.deltaTime);
    }

    void OnDrawGizmos()
    {
        if (DrawBVH && _lbvhNodesDebug.IsCreated) MortonDebugGizmos.Draw(this);
        if (DrawConvex && !CollisionPairs.IsEmpty)
            foreach (var pair in ConvexesDic)
            {
                if (!pair.Value.Item1.IsCreated() || !pair.Value.Item2.IsCreated()) continue;
                ConvexDebugGizmos.Draw(pair.Value.Item1, Bodies[pair.Key.Item1].transform);
                ConvexDebugGizmos.Draw(pair.Value.Item2, Bodies[pair.Key.Item2].transform);
            }
        if (DrawCollision)
            foreach (var c in _debugContacts)
                CollisionDebugDizmos.DrawManifoldPoint(c.pointA, c.pointB, c.normal, c.depth);
    }

    void OnDestroy() => Dispose();

    void Dispose()
    {
        if (AABBs.IsCreated) AABBs.Dispose();
        if (Positions.IsCreated) Positions.Dispose();
        if (Scales.IsCreated) Scales.Dispose();
        if (LocalBounds.IsCreated) LocalBounds.Dispose();
        if (MortonCodes.IsCreated) MortonCodes.Dispose();
        if (Levels.IsCreated) Levels.Dispose();
        if (ObjectIndices.IsCreated) ObjectIndices.Dispose();
        if (CollisionPairs.IsCreated) CollisionPairs.Dispose();
        if (EpaResults.IsCreated) EpaResults.Dispose();
        if (Rotations.IsCreated) Rotations.Dispose();
        if (_lbvhNodesDebug.IsCreated) _lbvhNodesDebug.Dispose();
    }
}
