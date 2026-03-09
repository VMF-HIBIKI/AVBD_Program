using Unity.Mathematics;
using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
public class DetectionBody : MonoBehaviour
{
    public MeshFilter MeshFilter { get; private set; }
    public MeshRenderer MeshRenderer { get; private set; }

    [Header("Physical Properties")]
    public bool isStatic;
    public float mass = 1f;
    public float friction = 0.5f;

    [HideInInspector] public float3 position;
    [HideInInspector] public Quaternion rotation;
    [HideInInspector] public float3 velocity;
    [HideInInspector] public float3 prevVelocity;
    [HideInInspector] public float3 angularVelocity;
    [HideInInspector] public float3 scale;

    // Saved at start of each step by solver
    [HideInInspector] public float3 initialPosition;
    [HideInInspector] public Quaternion initialRotation;

    // Inertial prediction target (computed by solver)
    [HideInInspector] public float3 inertialPosition;
    [HideInInspector] public Quaternion inertialRotation;

    void Awake()
    {
        MeshFilter = GetComponent<MeshFilter>();
        MeshRenderer = GetComponent<MeshRenderer>();
        MakeMeshUnique();
        InitializePhysics();
        ProcessSystem.Register(this);
    }

    void MakeMeshUnique()
    {
        if (MeshFilter.sharedMesh)
            MeshFilter.sharedMesh = Instantiate(MeshFilter.sharedMesh);
    }

    void InitializePhysics()
    {
        position = transform.position;
        rotation = transform.rotation;
        velocity = float3.zero;
        prevVelocity = float3.zero;
        angularVelocity = float3.zero;
        scale = transform.localScale;
        initialPosition = position;
        initialRotation = rotation;
        inertialPosition = position;
        inertialRotation = rotation;
    }

    public float3 GetHalfExtents()
    {
        if (MeshFilter == null || MeshFilter.sharedMesh == null)
            return new float3(0.5f);
        return (float3)MeshFilter.sharedMesh.bounds.extents;
    }

    void OnDestroy()
    {
        ProcessSystem.Unregister(this);
    }
}
