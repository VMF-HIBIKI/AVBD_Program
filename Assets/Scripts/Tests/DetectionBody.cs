using System;
using System.Collections.Generic;
using AVBD;
using Collision;
using Unity.Mathematics;
using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
public class DetectionBody : MonoBehaviour
{
    public MeshFilter MeshFilter { get; private set; }
    public MeshRenderer MeshRenderer { get; private set; }
    
    [HideInInspector]public List<Force> Forces =new List<Force>();

    [Header("Physical Properties")]
    public bool isStatic;
    public float mass = 1f;
    public float friction = 0.5f;
    public float startK = 10f;
    
    [HideInInspector] public float3 position;
    [HideInInspector] public Quaternion rotation;

     public float3 velocity;
     private float3 preVelocity;
    [HideInInspector] public float3 angularVelocity ;
    [HideInInspector] public float3 scale;

    [HideInInspector] public float3 predictedPosition;      // ← for AVBD
    [HideInInspector] public Quaternion predictedRotation;  // ← for AVBD
    
    private Dictionary<int, Force> lastFrameForces = new Dictionary<int, Force>();
    

    
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
        {
            MeshFilter.sharedMesh = Instantiate(MeshFilter.sharedMesh);
        }
    }
    

    void InitializePhysics()
    {
        position = transform.position;
        rotation = transform.rotation;
        angularVelocity = float3.zero;
        predictedPosition = position;
        predictedRotation = rotation;
        scale = transform.localScale;
    }

    public void InitializePreVelocity(float gravity)
    {
        preVelocity = velocity + new float3(0,gravity*Time.deltaTime,0);
    }

    public void Prediction(float gravity)
    {
        float3 Gravity = new float3(0f, -gravity, 0f);
        float3 DeltaPosition = position + velocity * Time.deltaTime;
        float3 Acceleration = (velocity - preVelocity) / Time.deltaTime;
        float AccelerationWeight = (-Acceleration.y / math.abs(Gravity).y);
        
        predictedPosition = DeltaPosition + Gravity * Time.deltaTime * Time.deltaTime ;
        /*
        float dt = Time.deltaTime;
        float3 w = angularVelocity; // world-space angular velocity (rad/s)
        float wLen = math.length(w);
        if (wLen < 1e-8f)
        {
            // 无角速度，保持当前旋转
            predictedRotation = rotation;
        }
        else
        {
            float angle = wLen * dt;                // 旋转角度
            float3 axis = w / wLen;                 // 旋转轴
            float half = 0.5f * angle;
            float s = math.sin(half);
            Quaternion dq = new Quaternion(axis.x * s, axis.y * s, axis.z * s, math.cos(half)); // delta quaternion
            // dq * rotation （注意 Unity 四元数相乘：先应用 dq 再 rotation）
            predictedRotation = (dq * rotation);
            predictedRotation = NormalizeQuaternion(predictedRotation);
        }*/

        Quaternion InertialW = new Quaternion(angularVelocity.x, angularVelocity.y, angularVelocity.z, 0f);
        var size = 0.5f * Time.deltaTime;
        var temp = InertialW * rotation;
        temp = new Quaternion(size * temp.x, size * temp.y, size * temp.z, size*temp.w);
        var addTemp = new Quaternion(rotation.x + temp.x, rotation.y + temp.y, rotation.z + temp.z,
            rotation.w + temp.w);
        predictedRotation = math.normalize(addTemp);
    }
    

    public void AddForce(EPAResult result, bool isBodyB, float alpha,float gamma)
    {
        int otherId = isBodyB ? result.BodyA : result.BodyB;
        int contactHash = HashContact(otherId, result.ContactA, result.ContactB);
        var normal = isBodyB ? -result.Normal : result.Normal;
        var contactA = isBodyB ? result.ContactB : result.ContactA;
        var contactB = isBodyB ? result.ContactA : result.ContactB;
        Force force;
        if (lastFrameForces.TryGetValue(contactHash, out var oldForce))
        {
            force = oldForce;
            force.Contact.Initialize(normal, position, contactA, contactB);
            force.K = math.max(new float3(-startK,-startK,-startK),oldForce.K * gamma);
            if (oldForce.Contact.Stick)
            {
                force.Lambda = oldForce.Lambda; // 完全复用
            }
            else
            {
                force.Lambda = oldForce.Lambda * alpha * gamma; // 衰减旧力，避免积累错误
            }    // 🔥 热启动核心
            force.Friction = oldForce.Friction;
        }
        else
        {
            force = new Force();
            FContact contact = new FContact();
            contact.Initialize(normal, position, contactA, contactB);
            force.Initial(contact,new float3(-startK,-startK,-startK),friction,float3.zero);
            force.otherBodyId = otherId;
            Forces.Add(force);
        }
    }

    public void CalculateNewTransform(int Iteration,float Alpha,float Beta)
    {
        float powT = Time.deltaTime * Time.deltaTime;
        float3x3 M = Diagonal(mass, mass, mass);
        float3x3 lhs = M / powT;
        
        // 粗略立方体惯量张量
        var size = MeshFilter.sharedMesh.bounds.size;
        float3 I =(mass / 12f) * new float3(
            size.y * size.y + size.z * size.z,  // Ixx
            size.x * size.x + size.z * size.z,  // Iyy
            size.x * size.x + size.y * size.y   // Izz
        );
        float3x3 InertiaTensor = Diagonal(I.x, I.y, I.z);
        for (int x = 0; x < Iteration; x++)
        {
            float3 rhs = math.mul(lhs, predictedPosition - position);

            float3x3 R = new float3x3(rotation);
            float3x3 Rlhs = math.mul(math.mul(math.transpose(R), InertiaTensor), R);
            Rlhs /= powT;

            float3 Rrhs = AngularFormQuaternion(predictedRotation, rotation, Time.deltaTime);

            for (int i = 0; i < Forces.Count; i++)
            {
                Force force = Forces[i];
                FContact Cont = force.Contact;

                float3 f = math.clamp(force.K * force.C + force.Lambda, force.Fmin, force.Fmax);
                rhs += force.JA[0] * f.x + force.JA[1] * f.y + force.JA[2] * f.z;
                Rrhs += force.JW[0] * f.x + force.JW[1] * f.y + force.JW[2] * f.z;

                lhs += OuterProduct(force.JA[0], force.JA[0] * force.K.x) +
                       OuterProduct(force.JA[1], force.JA[1] * force.K.y) +
                       OuterProduct(force.JA[2], force.JA[2] * force.K.z);

                Rlhs += OuterProduct(force.JW[0], force.JW[0] * force.K.x) +
                        OuterProduct(force.JW[1], force.JW[1] * force.K.y) +
                        OuterProduct(force.JW[2], force.JW[2] * force.K.z);
            }

            float[] Arhs = { rhs[0], rhs[1], rhs[2], Rrhs[0], Rrhs[1], Rrhs[2] };
            float3x3 Identity = Diagonal(1 / powT, 1 / powT, 1 / powT);
            MakeFloat6x6(lhs, Identity, Identity, Rlhs, out float[,] ALhs);
            LDLSolve6x6(ALhs, Arhs, out float[] Result);

            predictedPosition -= new float3(Result[0], Result[1], Result[2]);
            
            Quaternion Deltaq = new Quaternion(0.5f * Result[3],0.5f *  Result[4], 0.5f * Result[5], 0.0f); //(LDLSolve(Rlhs, Rrhs),0.0);
            var tempRotation = new Quaternion(predictedRotation.x - Deltaq.x * predictedRotation.x,
                predictedRotation.y - Deltaq.y * predictedRotation.y,
                predictedRotation.z - Deltaq.z * predictedRotation.z,
                predictedRotation.w - Deltaq.w * predictedRotation.w);
            predictedRotation = math.normalize(tempRotation);
            /*
            Quaternion dq = new Quaternion(0.5f * Result[3], 0.5f * Result[4], 0.5f * Result[5], 0);
            Quaternion temp = new Quaternion(
                dq.w * predictedRotation.x + dq.x * predictedRotation.w + dq.y * predictedRotation.z - dq.z * predictedRotation.y,
                dq.w * predictedRotation.y - dq.x * predictedRotation.z + dq.y * predictedRotation.w + dq.z * predictedRotation.x,
                dq.w * predictedRotation.z + dq.x * predictedRotation.y - dq.y * predictedRotation.x + dq.z * predictedRotation.w,
                dq.w * predictedRotation.w - dq.x * predictedRotation.x - dq.y * predictedRotation.y - dq.z * predictedRotation.z
            );
            predictedRotation = math.normalize(new Quaternion(
            predictedRotation.x + temp.x,
            predictedRotation.y + temp.y,
            predictedRotation.z + temp.z,
            predictedRotation.w + temp.w
            )););*/
            
            for (int y = 0; y < Forces.Count; y++)
            {
                Force CenterCF = Forces[y];

                CenterCF.ComputeConstraint(Alpha, predictedPosition - position, CalculatorDeltaW());
                CenterCF.Lambda = math.clamp(CenterCF.K * CenterCF.C + CenterCF.Lambda, CenterCF.Fmin, CenterCF.Fmax);

                CenterCF.K  -= Beta * math.abs(CenterCF.C);

                Forces[y] = CenterCF;
            }
        }
        UpdateTransform();
        
    }
    
    private int HashContact(int otherId, float3 contactA, float3 contactB)
    {
        // 简单哈希：另一刚体ID + 接触点近似
        int hash = otherId;
        hash = (hash * 397) ^ (int)(contactA.x );
        hash = (hash * 397) ^ (int)(contactA.y );
        hash = (hash * 397) ^ (int)(contactA.z );
        hash = (hash * 397) ^ (int)(contactB.x );
        hash = (hash * 397) ^ (int)(contactB.y );
        hash = (hash * 397) ^ (int)(contactB.z );
        return hash;
    }

    
    public void UpdateTransform()
    {
        preVelocity = velocity;
        velocity = (predictedPosition -position)/Time.deltaTime;
        angularVelocity = CalculatorDeltaW() / Time.deltaTime;
        
        position = predictedPosition;
        rotation = predictedRotation;
        transform.position = position;
        transform.rotation = rotation;
        
        lastFrameForces.Clear();
        foreach (var f in Forces)
        {
            int hash = HashContact(f.otherBodyId, f.Contact.ContactWorldSpacePositionA, f.Contact.ContactWorldSpacePositionB);
            lastFrameForces[hash] = f;
        }

        Forces.Clear();
    }
    
    public void UpdateTransformAsNoForce()
    {
        preVelocity = velocity;
        velocity = (predictedPosition -position)/Time.deltaTime;
        angularVelocity = angularVelocity;
        
        position = predictedPosition;
        rotation = predictedRotation;
        transform.position = position;
        transform.rotation = rotation;
        
        lastFrameForces.Clear();
        foreach (var f in Forces)
        {
            int hash = HashContact(f.otherBodyId, f.Contact.ContactWorldSpacePositionA, f.Contact.ContactWorldSpacePositionB);
            lastFrameForces[hash] = f;
        }

        Forces.Clear();
    }

    float3 CalculatorDeltaW()
    {
        /*
        // qDelta = q_pred * inverse(q)
        Quaternion inv = Quaternion.Inverse(rotation);
        Quaternion qDelta = predictedRotation * inv;
        qDelta = NormalizeQuaternion(qDelta);

        // 确保数值稳定： clamp w
        qDelta.w = math.clamp(qDelta.w, -1f, 1f);

        // angle = 2 * acos(w)
        float angle = 2f * Mathf.Acos(qDelta.w);
        float s = math.sqrt(1f - qDelta.w * qDelta.w);

        if (s < 1e-6f)
        {
            // 角度很小，近似： vector part ≈ 0.5 * axis * angle
            return 2f * new float3(qDelta.x, qDelta.y, qDelta.z);
        }
        else
        {
            float3 axis = new float3(qDelta.x / s, qDelta.y / s, qDelta.z / s);
            return axis * angle;
        }*/

        Quaternion invRotation = math.inverse(rotation);
        float3 DeltaW = new float3(2 * predictedRotation.x * invRotation.x, 2 * predictedRotation.y * invRotation.y,
            2 * predictedRotation.z * invRotation.z);

        return DeltaW;
    }
    
    float3x3 Diagonal(float m00, float m11, float m22)
    {
        return new float3x3(
            m00, 0, 0,
            0, m11, 0,
            0, 0, m22
        );
    }
    
    float3 AngularFormQuaternion(Quaternion q, Quaternion qPrev, float deltaTime)
    {
        // 计算从 qPrev -> q 的角速度估计：
        // delta quaternion = q * inverse(qPrev) ，
        // 然后将 delta 转成 axis * angle / deltaTime
        /*
        Quaternion invPrev = Quaternion.Inverse(qPrev);
        Quaternion qDelta = q * invPrev;
        qDelta = NormalizeQuaternion(qDelta);
        qDelta.w = math.clamp(qDelta.w, -1f, 1f);

        float angle = 2f * Mathf.Acos(qDelta.w);
        float s = math.sqrt(1f - qDelta.w * qDelta.w);
        if (s < 1e-6f)
        {
            // 小角度近似
            return 2f * new float3(qDelta.x, qDelta.y, qDelta.z) / Math.Max(deltaTime, 1e-9f);
        }
        float3 axis = new float3(qDelta.x / s, qDelta.y / s, qDelta.z / s);
        return axis * angle / Math.Max(deltaTime, 1e-9f);*/
        float3 w = 0.0f;
        w.x = qPrev.w * q.x - qPrev.x * q.w - qPrev.y * q.z + qPrev.z * q.y;
        w.y = qPrev.w * q.y + qPrev.x * q.z - qPrev.y * q.w - qPrev.z * q.x;
        w.z = qPrev.w * q.z - qPrev.x * q.y + qPrev.y * q.y - qPrev.z * q.w;

        return 2 / deltaTime * w;
        
    }
    
    float3x3 OuterProduct(float3 a, float3 b)
    {
        return new float3x3(b * a.x, b * a.y, b * a.z);
    }
    
    void MakeFloat6x6(float3x3 A,float3x3 B,float3x3 C,float3x3 D,out float[,] OutValue)
    {
        OutValue = new float[6,6];
        float[] C1 = {A[0][0],A[0][1],A[0][2],B[0][0],B[0][1],B[0][2]};
        float[] C2 = {A[1][0],A[1][1],A[1][2],B[1][0],B[1][1],B[1][2]};
        float[] C3 = {A[2][0],A[2][1],A[2][2],B[2][0],B[2][1],B[2][2]};
        float[] C4 = {C[0][0],C[0][1],C[0][2],D[0][0],D[0][1],D[0][2]};
        float[] C5 = {C[1][0],C[1][1],C[1][2],D[1][0],D[1][1],D[1][2]};
        float[] C6 = {C[2][0],C[2][1],C[2][2],D[2][0],D[2][1],D[2][2]};

        OutValue[0,0] = C1[0];
        OutValue[0,1] = C1[1];
        OutValue[0,2] = C1[2];
        OutValue[0,3] = C1[3];
        OutValue[0,4] = C1[4];
        OutValue[0,5] = C1[5];
        
        OutValue[1,0] = C2[0];
        OutValue[1,1] = C2[1];
        OutValue[1,2] = C2[2];
        OutValue[1,3] = C2[3];
        OutValue[1,4] = C2[4];
        OutValue[1,5] = C2[5];
        
        OutValue[2,0] = C3[0];
        OutValue[2,1] = C3[1];
        OutValue[2,2] = C3[2];
        OutValue[2,3] = C3[3];
        OutValue[2,4] = C3[4];
        OutValue[2,5] = C3[5];
        
        OutValue[3,0] = C4[0];
        OutValue[3,1] = C4[1];
        OutValue[3,2] = C4[2];
        OutValue[3,3] = C4[3];
        OutValue[3,4] = C4[4];
        OutValue[3,5] = C4[5];
        
        OutValue[4,0] = C5[0];
        OutValue[4,1] = C5[1];
        OutValue[4,2] = C5[2];
        OutValue[4,3] = C5[3];
        OutValue[4,4] = C5[4];
        OutValue[4,5] = C5[5];
        
        OutValue[5,0] = C6[0];
        OutValue[5,1] = C6[1];
        OutValue[5,2] = C6[2];
        OutValue[5,3] = C6[3];
        OutValue[5,4] = C6[4];
        OutValue[5,5] = C6[5];
    }
    
    void LDLSolve6x6(float[,] lhs,float[] rhs,out float[] OutValue)
    {
        OutValue = new float[6];
        float[,] L = new float[6,6];
        float[] D = new float[6] ;
        for (int i = 0; i < 6; i++) {
            // compute D[i]
            float sum = 0.0f;
            for (int j = 0; j < i; j++) sum += L[i,j] * L[i,j] * D[j];
            D[i] = lhs[i,i] - sum;

            L[i,i] = 1.0f;

            // compute L[j][i] for j > i
            for (int j = i + 1; j < 6; j++) {
                float s = lhs[j,i];
                for (int k = 0; k < i; k++) s -= L[j,k] * L[i,k] * D[k];
                L[j,i] = s / D[i];
            }
        }

        float[] y = new float[6];
        for (int i = 0; i < 6; i++) {
            float sum = 0.0f;
            for (int j = 0; j < i; j++) sum += L[i,j] * y[j];
            y[i] = rhs[i] - sum;
        }

        // diagonal solve: solve Dz = y
        float[] z = new float[6];
        for (int i = 0; i < 6; i++) z[i] = y[i] / D[i];

        // backward substitution: solve L^T x = z
        float[] x = new float[6];
        for (int i = 5; i >= 0; i--) {
            float sum = 0.0f;
            for (int j = i + 1; j < 6; j++) sum += L[j,i] * x[j];
            x[i] = z[i] - sum;
        }

        OutValue = x;
    }
    
    private void OnDestroy()
    {
        ProcessSystem.Unregister(this);
    }
}
