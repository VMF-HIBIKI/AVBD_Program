using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using Collision;
using Convex;
using Unity.VisualScripting;
using Utils;

public class ProcessSystem : MonoBehaviour
{
    [Header("Settings")]
    [Range(2,20)]
    public int MaxDepth = 5;
    public bool DrawOctree = true;
    public bool DrawConvex = true;
    public bool DrawCollision = true;

    [SerializeField] [Range(0f,1f)] private float alpha ;
    //缩放参数
    [SerializeField] [Range(0f,0.99f)]private float gamma;
    [SerializeField] [Range(0.01f,0.99f)]private float friction;
    [SerializeField] [Range(10f, 100000f)] private float beta;
    public float Gravity = 9.8f;
    [Range(1,150)] public int ConstraintSolverIteratorCount = 2;
    

    private List<ulong> Granularitys = new List<ulong>();
    private List<ulong> BitMasks = new List<ulong>();

    private AABB _sceneBounds;
    private static List<DetectionBody> Bodies = new List<DetectionBody>();
    private static Dictionary<Tuple<int,int>,Tuple<NativeConvex,NativeConvex>> ConvexesDic = new Dictionary<Tuple<int, int>, Tuple<NativeConvex, NativeConvex>>();
    
    // NativeArray 数据
    private NativeArray<AABB> AABBs;
    private NativeArray<float3> Positions;//每个Body的中心位置
    private NativeArray<quaternion>  Rotations;
    private NativeArray<float3> Scales;
    private NativeArray<Bounds> LocalBounds;
    private NativeArray<ulong> MortonCodes;
    private NativeArray<int> Levels;
    private NativeArray<int> ObjectIndices;
    private NativeList<int2> CollisionPairs;
    private NativeList<EPAResult> EpaResults;

    
    public AABB SceneBounds => _sceneBounds;
    public NativeList<int2> CollisionPairsRead => CollisionPairs;
    public NativeArray<AABB> AABBsRead => AABBs;
    public NativeArray<int> LevelsRead => Levels;
    public int ObjectCount => Bodies.Count;

    private int capacity = 1024;

    void Awake()
    {
        InitializeBroadPhaseContext();
    }

    private void Start()
    {
        foreach (var body in Bodies)
        {
            if(!body.isStatic)
                body.InitializePreVelocity(Gravity);
        }
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
            // 空场景默认正方体边界（1x1x1）
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

            var bounds = mesh.bounds;
            // 计算物体在世界空间的真实边界（考虑旋转和缩放）
            var worldMin = math.mul(body.transform.localToWorldMatrix, new float4(bounds.min, 1f)).xyz;
            var worldMax = math.mul(body.transform.localToWorldMatrix, new float4(bounds.max, 1f)).xyz;

            min = math.min(min, worldMin);
            max = math.max(max, worldMax);
        }

        // 计算原始边界的中心和尺寸
        float3 center = (min + max) * 0.5f;
        float3 originalSize = max - min;
        // 找到最大边长（正方体的边长）
        float maxExtent = math.cmax(originalSize) * 0.5f; // 半边长

        // 扩展边界为正方体（确保包含所有物体）
        _sceneBounds = new AABB
        {
            Min = center - new float3(maxExtent),
            Max = center + new float3(maxExtent)
        };

        // 可选：添加一点padding避免物体刚好贴边
        float padding = 0.1f;
        _sceneBounds.Min -= new float3(padding);
        _sceneBounds.Max += new float3(padding);
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

    private void Update()
    {
        if(EpaResults.IsCreated)EpaResults.Clear();
        UpdateBroadPhaseProcess();
        CreatConvexesByPairs();
        NarrowPhaseProcess();
        UpdateBodies();
    }

    private void LateUpdate()
    {
        LateUpdateBodies();
    }

    private void UpdateBroadPhaseProcess()
    {
        if (Bodies.Count == 0) return;
        if (Bodies.Count > AABBs.Length) Allocate(Bodies.Count * 2);

        RefreshSceneBounds();

        // 更新物体数据
        for (int i = 0; i < Bodies.Count; i++)
        {
            var t = Bodies[i].transform;
            Positions[i] = t.position;
            Rotations[i] = t.rotation;
            Scales[i] = t.lossyScale;
            LocalBounds[i] = Bodies[i].MeshFilter.sharedMesh.bounds;
            ObjectIndices[i] = i;
        }

        // Step1: Mesh → AABB
        var meshJob = new MeshToAABBJob
        {
            Positions = Positions,
            Rotations = Rotations,
            Scales = Scales,
            LocalBounds = LocalBounds,
            WorldAABBs = AABBs
        }.Schedule(Bodies.Count, 64);


        // Step2: Morton编码
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

        // Step3: Morton排序
        var sortJob = new SortJob
        {
            MortonCodes = MortonCodes,
            ObjectIndices = ObjectIndices
        }.Schedule(mortonJob);
        // Step4: BroadPhase检测
        CollisionPairs.Clear();
        var broadJob = new BroadPhaseJob
        {
            AABBs = AABBs,
            MortonCodes = MortonCodes,
            ObjectIndices = ObjectIndices,
            Levels = Levels,
            MaxDepth = MaxDepth,
            Count = Bodies.Count,
            CollisionPairs = CollisionPairs.AsParallelWriter()
        }.Schedule(Bodies.Count, 2, sortJob);
        broadJob.Complete();
        
        for (int i = 0; i < CollisionPairs.Length; i++)
        {
            var pair = CollisionPairs[i];
            var a = Bodies[pair.x];
            var b = Bodies[pair.y];
            //Debug.Log($"CollisionPair: {a.name} <-> {b.name}");
        }
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
            if(convexes.Item1.IsCreated())convexes.Item1.Dispose();
            if(convexes.Item2.IsCreated())convexes.Item2.Dispose();
        }

        ConvexesDic.Clear();
        
        foreach (var Pair in CollisionPairs)
        {
            var pairA = ConvexConstructor.CreatConvex(Bodies[Pair.x]);
            var pairB =ConvexConstructor.CreatConvex(Bodies[Pair.y]);
            Tuple<int, int> key = new Tuple<int, int>(Pair.x, Pair.y);
            Tuple<int, int> inverseKey = new Tuple<int, int>(Pair.y, Pair.x);
            if(ConvexesDic.ContainsKey(key)||ConvexesDic.ContainsKey(inverseKey)) {continue;}
            ConvexesDic[key] = new Tuple<NativeConvex, NativeConvex>(pairA, pairB);
        }
    }

    private void NarrowPhaseProcess()
    {
        //生命周期可能要改
        if (EpaResults.IsCreated)
        {
            EpaResults.Dispose();
        }
        EpaResults = new NativeList<EPAResult>(ConvexesDic.Count,Allocator.Persistent);
        foreach (var Pair in ConvexesDic)
        {
            var convexPair = Pair.Value;
            if (GJK_EPA.DetectedCollisionAndResolve(convexPair.Item1, convexPair.Item2, out var result1))
            {
                result1.BodyA = Pair.Key.Item1;
                result1.BodyB = Pair.Key.Item2;
                var manifolds = new NativeList<ManifoldPoint>(4, Allocator.Temp);
                GJK_EPA.BuildManifold(convexPair.Item1, convexPair.Item2, result1.Normal, ref manifolds);
                /*
                if (result2.Depth >= 0.9 * result1.Depth)
                {
                    EpaResults.Add(result1);
                    Bodies[result1.BodyA].AddForce(result1,false,alpha,gamma);
                    Bodies[result1.BodyB].AddForce(result1,true,alpha,gamma);

                    Debug.Log(result1.ContactA.xyz+","+result1.ContactB.xyz);
                    Debug.Log(result1.Normal.xyz);
                }
                else
                {   
                    EpaResults.Add(result2);
                    Bodies[result2.BodyA].AddForce(result2,false,alpha,gamma);
                    Bodies[result2.BodyB].AddForce(result2,true,alpha,gamma);

                    Debug.Log(result2.ContactB.xyz+","+result2.ContactA.xyz);
                    Debug.Log(result2.Normal.xyz);
                }*/
                if (manifolds.Length > 0)
                {
                    // 使用 Manifold 中的每一个点施加力
                    foreach (var point in manifolds)
                    {
                        // 构造一个临时的 EPAResult 传给你的 AddForce (或者修改 AddForce 接受 ManifoldPoint)
                        EPAResult tempResult = new EPAResult();
                        tempResult.BodyA = result1.BodyA;
                        tempResult.BodyB = result1.BodyB;
                        tempResult.Normal = result1.Normal;
                        tempResult.Depth = point.Depth;       // 使用流形点的深度
                        tempResult.ContactA = point.ContactA; // 使用流形点 A
                        tempResult.ContactB = point.ContactB; // 使用流形点 B
                        if (!Bodies[result1.BodyA].isStatic)
                        {
                            Bodies[result1.BodyA].AddForce(tempResult, false, alpha, gamma);
                        }

                        if (!Bodies[result1.BodyB].isStatic)
                        {
                            Bodies[result1.BodyB].AddForce(tempResult, true, alpha, gamma);
                        }
                    }
                    Debug.Log("FocesCount:"+Bodies[result1.BodyA].name+" "+Bodies[result1.BodyA].Forces.Count);
                    Debug.Log("FocesCount:"+Bodies[result1.BodyB].name+" "+Bodies[result1.BodyB].Forces.Count);
                }
                else
                {
                    // 如果裁剪失败（极少数情况），回退使用 EPA 的单点结果
                    Bodies[result1.BodyA].AddForce(result1, false, alpha, gamma);
                    Bodies[result1.BodyB].AddForce(result1, true, alpha, gamma);
                }
    
                manifolds.Dispose();
            }
            else
            {
                result1 = new EPAResult();
                result1 = default;
                EpaResults.Add(result1);
            }
        }
        
    }

    private void UpdateBodies()
    {
        foreach (var body in Bodies)
        {
            if(body.isStatic) continue;
            body.Prediction(Gravity);
            if (body.Forces.Count > 0)
            {
               body.CalculateNewTransform(ConstraintSolverIteratorCount, alpha, beta); 
            }
            else
            {
                body.UpdateTransformAsNoForce();
            }
        }
        
    }

    private void LateUpdateBodies()
    {
        foreach (var body in Bodies)
        {
            body.Forces.Clear();
        }
    }
    
    void OnDrawGizmos()
    {
        if (DrawOctree && AABBs.IsCreated) MortonDebugGizmos.Draw(this);
        if (DrawConvex && !CollisionPairs.IsEmpty)
            foreach (var pair in ConvexesDic)
            {
                if(!pair.Value.Item1.IsCreated() || !pair.Value.Item2.IsCreated()) continue;
                ConvexDebugGizmos.Draw(pair.Value.Item1,Bodies[pair.Key.Item1].transform);
                ConvexDebugGizmos.Draw(pair.Value.Item2,Bodies[pair.Key.Item2].transform);
            }
        if (DrawCollision && EpaResults.IsCreated)
            foreach (var result in EpaResults)
            {
                if(result.HasResult) CollisionDebugDizmos.Draw(result);
            }
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
    }
}
