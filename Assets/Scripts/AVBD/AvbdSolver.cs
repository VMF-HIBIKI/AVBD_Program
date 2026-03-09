using System;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace AVBD
{
    // ======================== Blittable Data ========================

    public struct AvbdBodyData
    {
        public float3 position;
        public quaternion rotation;
        public float3 velocity;
        public float3 angularVelocity;
        public float3 prevVelocity;
        public float3 initialPosition;
        public quaternion initialRotation;
        public float3 inertialPosition;
        public quaternion inertialRotation;
        public float mass; // 0 → static
        public float3 halfExtents;
    }

    public struct AvbdContact
    {
        public int bodyA;
        public int bodyB; // -1 = static

        public float3 normal;
        public float3 rA;
        public float3 rB;
        public float depth;
        public float friction;

        public float3 JAn_lin, JAn_ang;
        public float3 JBn_lin, JBn_ang;
        public float3 JAt1_lin, JAt1_ang;
        public float3 JBt1_lin, JBt1_ang;
        public float3 JAt2_lin, JAt2_ang;
        public float3 JBt2_lin, JBt2_ang;

        public float3 C;
        public float3 C0;
        public float3 lambda;
        public float3 penalty;
        public float3 fmin, fmax;
    }

    public unsafe struct Float6x6
    {
        public fixed float m[36];
    }

    public unsafe struct Float6
    {
        public fixed float v[6];
    }

    struct CachedDual
    {
        public float3 lambda;
        public float3 penalty;
    }

    // ======================== Static Math ========================

    static class SM
    {
        public static float3 DeltaW(quaternion to, quaternion from)
        {
            quaternion dq = math.mul(to, math.conjugate(from));
            if (dq.value.w < 0) dq.value = -dq.value;
            return dq.value.xyz * 2f;
        }

        public static float Dot6(float3 la, float3 aa, float3 lb, float3 ab)
        {
            return math.dot(la, lb) + math.dot(aa, ab);
        }

        public static void GetJ(in AvbdContact c, bool isA, int axis,
                                 out float3 lin, out float3 ang)
        {
            lin = float3.zero; ang = float3.zero;
            if (isA)
            {
                if (axis == 0) { lin = c.JAn_lin; ang = c.JAn_ang; }
                else if (axis == 1) { lin = c.JAt1_lin; ang = c.JAt1_ang; }
                else { lin = c.JAt2_lin; ang = c.JAt2_ang; }
            }
            else
            {
                if (axis == 0) { lin = c.JBn_lin; ang = c.JBn_ang; }
                else if (axis == 1) { lin = c.JBt1_lin; ang = c.JBt1_ang; }
                else { lin = c.JBt2_lin; ang = c.JBt2_ang; }
            }
        }

        public static void ComputeConstraintFull(
            ref AvbdContact c, in NativeArray<AvbdBodyData> bodies, float alpha)
        {
            var bA = bodies[c.bodyA];
            bool bStatic = c.bodyB < 0;

            float3 rAw = math.rotate(bA.rotation, c.rA);
            float3 rBw = bStatic ? float3.zero
                                 : math.rotate(bodies[c.bodyB].rotation, c.rB);

            float3 t1, t2;
            if (math.abs(c.normal.y) > 0.9f)
                t1 = math.normalizesafe(math.cross(c.normal, new float3(1, 0, 0)));
            else
                t1 = math.normalizesafe(math.cross(c.normal, new float3(0, 1, 0)));
            t2 = math.cross(c.normal, t1);

            c.JAn_lin  = c.normal;    c.JAn_ang  = math.cross(rAw, c.normal);
            c.JAt1_lin = t1;          c.JAt1_ang = math.cross(rAw, t1);
            c.JAt2_lin = t2;          c.JAt2_ang = math.cross(rAw, t2);

            if (bStatic)
            {
                c.JBn_lin = c.JBn_ang = float3.zero;
                c.JBt1_lin = c.JBt1_ang = float3.zero;
                c.JBt2_lin = c.JBt2_ang = float3.zero;
            }
            else
            {
                c.JBn_lin  = -c.normal;  c.JBn_ang  = -math.cross(rBw, c.normal);
                c.JBt1_lin = -t1;        c.JBt1_ang = -math.cross(rBw, t1);
                c.JBt2_lin = -t2;        c.JBt2_ang = -math.cross(rBw, t2);
            }

            float3 dpA = bA.position - bA.initialPosition;
            float3 dwA = DeltaW(bA.rotation, bA.initialRotation);
            float3 dpB = float3.zero, dwB = float3.zero;
            if (!bStatic)
            {
                var bB = bodies[c.bodyB];
                dpB = bB.position - bB.initialPosition;
                dwB = DeltaW(bB.rotation, bB.initialRotation);
            }

            c.C = new float3(
                c.C0.x * (1f - alpha) + Dot6(c.JAn_lin, c.JAn_ang, dpA, dwA)
                                      + Dot6(c.JBn_lin, c.JBn_ang, dpB, dwB),
                c.C0.y * (1f - alpha) + Dot6(c.JAt1_lin, c.JAt1_ang, dpA, dwA)
                                      + Dot6(c.JBt1_lin, c.JBt1_ang, dpB, dwB),
                c.C0.z * (1f - alpha) + Dot6(c.JAt2_lin, c.JAt2_ang, dpA, dwA)
                                      + Dot6(c.JBt2_lin, c.JBt2_ang, dpB, dwB));

            float fb = math.abs(c.lambda.x) * c.friction;
            c.fmin = new float3(-1e30f, -fb, -fb);
            c.fmax = new float3(0, fb, fb);
        }

        public static void ComputeC0(
            ref AvbdContact c, in NativeArray<AvbdBodyData> bodies)
        {
            var bA = bodies[c.bodyA];
            float3 wA = bA.position + math.rotate(bA.rotation, c.rA);
            float3 wB = c.bodyB >= 0
                ? bodies[c.bodyB].position + math.rotate(bodies[c.bodyB].rotation, c.rB)
                : c.rB;
            c.C0 = new float3(math.dot(wA - wB, c.normal) - c.depth, 0, 0);
        }

        public static unsafe void SolveLDLT(
            ref Float6x6 A, ref Float6 b, ref Float6 result)
        {
            const float reg = 1e-12f;
            Float6x6 L = default;
            Float6 D = default;

            for (int i = 0; i < 6; i++)
            {
                float sum = 0;
                for (int j = 0; j < i; j++)
                    sum += L.m[i * 6 + j] * L.m[i * 6 + j] * D.v[j];
                D.v[i] = A.m[i * 6 + i] - sum;
                if (math.abs(D.v[i]) < reg)
                    D.v[i] = D.v[i] >= 0 ? reg : -reg;
                L.m[i * 6 + i] = 1f;
                for (int j = i + 1; j < 6; j++)
                {
                    float s = A.m[j * 6 + i];
                    for (int k = 0; k < i; k++)
                        s -= L.m[j * 6 + k] * L.m[i * 6 + k] * D.v[k];
                    L.m[j * 6 + i] = s / D.v[i];
                }
            }

            Float6 y = default;
            for (int i = 0; i < 6; i++)
            {
                float s = 0;
                for (int j = 0; j < i; j++) s += L.m[i * 6 + j] * y.v[j];
                y.v[i] = b.v[i] - s;
            }
            Float6 z = default;
            for (int i = 0; i < 6; i++) z.v[i] = y.v[i] / D.v[i];

            for (int i = 5; i >= 0; i--)
            {
                float s = 0;
                for (int j = i + 1; j < 6; j++) s += L.m[j * 6 + i] * result.v[j];
                result.v[i] = z.v[i] - s;
            }
        }
    }

    // ======================== Jobs ========================

    [BurstCompile]
    struct InitBodiesJob : IJobParallelFor
    {
        public NativeArray<AvbdBodyData> bodies;
        [ReadOnly] public NativeArray<int> bcOffsets;
        public float3 gravity;
        public float dt;

        public void Execute(int bi)
        {
            var b = bodies[bi];
            if (b.mass <= 0) return;
            b.initialPosition = b.position;
            b.initialRotation = b.rotation;
            float dt2 = dt * dt;
            b.inertialPosition = b.position + b.velocity * dt + gravity * dt2;
            quaternion wq = new quaternion(b.angularVelocity.x,
                                           b.angularVelocity.y,
                                           b.angularVelocity.z, 0f);
            float4 rd = math.mul(wq, b.rotation).value;
            b.inertialRotation = new quaternion(math.normalizesafe(
                b.rotation.value + 0.5f * dt * rd,
                new float4(0, 0, 0, 1)));

            bool hasContacts = bcOffsets[bi + 1] > bcOffsets[bi];
            if (hasContacts)
            {
                float invDt = 1f / dt;
                float3 accel = (b.velocity - b.prevVelocity) * invDt;
                float gravLen = math.length(gravity);
                float accelWeight = 0f;
                if (gravLen > 1e-6f)
                {
                    float3 gravDir = gravity / gravLen;
                    accelWeight = math.clamp(
                        math.dot(accel, gravDir) / gravLen, 0f, 1f);
                }
                b.position = b.initialPosition + b.velocity * dt
                           + gravity * (accelWeight * dt2);
            }
            else
            {
                b.position = b.inertialPosition;
            }
            b.rotation = b.inertialRotation;
            bodies[bi] = b;
        }
    }

    [BurstCompile]
    struct PenaltyFloorJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<AvbdBodyData> bodies;
        public NativeArray<AvbdContact> contacts;
        public float penaltyMin;
        public float penaltyScale;
        public float invDt2;

        public void Execute(int ci)
        {
            var c = contacts[ci];
            float floor = math.max(penaltyMin,
                                   penaltyScale * bodies[c.bodyA].mass * invDt2);
            c.penalty = math.max(c.penalty, new float3(floor));
            contacts[ci] = c;
        }
    }

    [BurstCompile]
    struct ComputeC0Job : IJobParallelFor
    {
        [ReadOnly] public NativeArray<AvbdBodyData> bodies;
        public NativeArray<AvbdContact> contacts;

        public void Execute(int ci)
        {
            var c = contacts[ci];
            SM.ComputeC0(ref c, bodies);
            contacts[ci] = c;
        }
    }

    [BurstCompile]
    struct ComputeConstraintsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<AvbdBodyData> bodies;
        public NativeArray<AvbdContact> contacts;
        public float alpha;

        public void Execute(int ci)
        {
            var c = contacts[ci];
            SM.ComputeConstraintFull(ref c, bodies, alpha);
            contacts[ci] = c;
        }
    }

    [BurstCompile]
    struct PrimalUpdateJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<int> bodyIndices;
        [NativeDisableParallelForRestriction]
        public NativeArray<AvbdBodyData> bodies;
        [ReadOnly] public NativeArray<AvbdContact> contacts;
        [ReadOnly] public NativeArray<int> bcOffsets;
        [ReadOnly] public NativeArray<int> bcIndices;
        public float invDt2;
        public float alpha;

        public unsafe void Execute(int i)
        {
            int bi = bodyIndices[i];
            var body = bodies[bi];
            if (body.mass <= 0) return;

            Float6x6 lhs = default;
            Float6 rhs = default;

            float m = body.mass * invDt2;
            lhs.m[0] = m; lhs.m[7] = m; lhs.m[14] = m;
            float sx = 2f * body.halfExtents.x;
            float sy = 2f * body.halfExtents.y;
            float sz = 2f * body.halfExtents.z;
            lhs.m[21] = body.mass / 12f * (sy * sy + sz * sz) * invDt2;
            lhs.m[28] = body.mass / 12f * (sx * sx + sz * sz) * invDt2;
            lhs.m[35] = body.mass / 12f * (sx * sx + sy * sy) * invDt2;

            float3 dPos = body.position - body.inertialPosition;
            float3 dW = SM.DeltaW(body.rotation, body.inertialRotation);
            rhs.v[0] = lhs.m[0]  * dPos.x;
            rhs.v[1] = lhs.m[7]  * dPos.y;
            rhs.v[2] = lhs.m[14] * dPos.z;
            rhs.v[3] = lhs.m[21] * dW.x;
            rhs.v[4] = lhs.m[28] * dW.y;
            rhs.v[5] = lhs.m[35] * dW.z;

            float boostFloor = 0.005f * body.mass * invDt2;

            int cStart = bcOffsets[bi];
            int cEnd   = bcOffsets[bi + 1];
            for (int ci = cStart; ci < cEnd; ci++)
            {
                AvbdContact c = contacts[bcIndices[ci]];
                SM.ComputeConstraintFull(ref c, bodies, alpha);
                bool isA = c.bodyA == bi;

                for (int axis = 0; axis < 3; axis++)
                {
                    SM.GetJ(in c, isA, axis, out float3 jL, out float3 jA);
                    Float6 J = default;
                    J.v[0] = jL.x; J.v[1] = jL.y; J.v[2] = jL.z;
                    J.v[3] = jA.x; J.v[4] = jA.y; J.v[5] = jA.z;

                    float pen = math.max(c.penalty[axis], boostFloor);
                    float f = math.clamp(
                        pen * c.C[axis] + c.lambda[axis],
                        c.fmin[axis], c.fmax[axis]);

                    for (int r = 0; r < 6; r++)
                    {
                        rhs.v[r] += J.v[r] * f;
                        for (int cc = 0; cc < 6; cc++)
                            lhs.m[r * 6 + cc] += J.v[r] * J.v[cc] * pen;
                    }
                }
            }

            Float6 delta = default;
            SM.SolveLDLT(ref lhs, ref rhs, ref delta);

            body.position -= new float3(delta.v[0], delta.v[1], delta.v[2]);

            quaternion dq = new quaternion(delta.v[3], delta.v[4], delta.v[5], 0);
            float4 tmp = math.mul(dq, body.rotation).value;
            body.rotation = new quaternion(math.normalizesafe(
                body.rotation.value - 0.5f * tmp,
                new float4(0, 0, 0, 1)));

            bodies[bi] = body;
        }
    }

    [BurstCompile]
    struct DualUpdateJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<AvbdBodyData> bodies;
        public NativeArray<AvbdContact> contacts;
        public float alpha;
        public float beta;
        public float penaltyMax;

        public void Execute(int ci)
        {
            var c = contacts[ci];
            SM.ComputeConstraintFull(ref c, bodies, alpha);

            float3 raw = c.penalty * c.C + c.lambda;
            float3 nl = math.clamp(raw, c.fmin, c.fmax);
            float3 inside = new float3(
                (nl.x > c.fmin.x && nl.x < c.fmax.x) ? 1f : 0f,
                (nl.y > c.fmin.y && nl.y < c.fmax.y) ? 1f : 0f,
                (nl.z > c.fmin.z && nl.z < c.fmax.z) ? 1f : 0f);
            c.penalty = math.min(
                c.penalty + inside * beta * math.abs(c.C),
                new float3(penaltyMax));
            c.lambda = nl;
            contacts[ci] = c;
        }
    }

    [BurstCompile]
    struct VelocityUpdateJob : IJobParallelFor
    {
        public NativeArray<AvbdBodyData> bodies;
        public float invDt;

        public void Execute(int bi)
        {
            var b = bodies[bi];
            if (b.mass <= 0) return;
            b.prevVelocity = b.velocity;
            b.velocity = (b.position - b.initialPosition) * invDt;
            quaternion dq = math.mul(b.rotation, math.conjugate(b.initialRotation));
            if (dq.value.w < 0) dq.value = -dq.value;
            b.angularVelocity = dq.value.xyz * (2f * invDt);
            bodies[bi] = b;
        }
    }

    // ======================== Solver ========================

    public class AvbdSolver
    {
        public float3 gravity = new float3(0, -9.8f, 0);
        public int iterations = 10;
        public float alpha = 0.95f;
        public float beta  = 1000f;
        public float gamma = 0.99f;

        const float PenaltyMin   = 1000f;
        const float PenaltyMax   = 1e9f;
        const float PenaltyScale = 0.25f;

        List<DetectionBody> bodies;
        List<AvbdContact> contacts = new List<AvbdContact>();
        Dictionary<long, CachedDual> dualCache = new Dictionary<long, CachedDual>();

        public void SetBodies(List<DetectionBody> bodyList) { bodies = bodyList; }
        public void ClearContacts() { contacts.Clear(); }

        public void AddContact(int bodyA, int bodyB, float3 normal,
                               float3 worldContactA, float3 worldContactB,
                               float depth, float fric)
        {
            var bA = bodies[bodyA];
            float3 rA_local = math.mul(math.transpose(new float3x3(bA.rotation)),
                                        worldContactA - bA.position);
            float3 rB_local;
            if (bodyB >= 0 && bodyB < bodies.Count && !bodies[bodyB].isStatic)
            {
                var bB = bodies[bodyB];
                rB_local = math.mul(math.transpose(new float3x3(bB.rotation)),
                                    worldContactB - bB.position);
            }
            else
            {
                rB_local = worldContactB;
                bodyB = -1;
            }

            AvbdContact c = default;
            c.bodyA = bodyA;  c.bodyB = bodyB;
            c.normal = math.normalizesafe(normal, new float3(0, 1, 0));
            c.rA = rA_local;  c.rB = rB_local;
            c.depth = depth;   c.friction = fric;
            c.lambda = float3.zero;
            c.penalty = new float3(PenaltyMin);
            c.fmin = new float3(-1e30f, 0, 0);
            c.fmax = float3.zero;

            long key = MakeKey(bodyA, bodyB, worldContactA, c.normal);
            if (dualCache.TryGetValue(key, out var cached))
            {
                c.lambda  = cached.lambda * alpha * gamma;
                c.penalty = math.max(new float3(PenaltyMin),
                            math.min(new float3(PenaltyMax), cached.penalty * gamma));
            }
            contacts.Add(c);
        }

        // -------------------- Step --------------------

        public void Step(float dt)
        {
            if (bodies == null || bodies.Count == 0) return;
            int bCount = bodies.Count;
            int cCount = contacts.Count;
            float invDt  = 1f / dt;
            float invDt2 = 1f / (dt * dt);

            var bodyData    = new NativeArray<AvbdBodyData>(bCount, Allocator.TempJob);
            var contactData = new NativeArray<AvbdContact>(math.max(cCount, 1), Allocator.TempJob);
            CopyBodiesToNative(bodyData);
            for (int i = 0; i < cCount; i++) contactData[i] = contacts[i];

            BuildBodyContactMap(bCount, contactData, cCount,
                                out var bcOff, out var bcIdx);
            BuildColoring(bCount, contactData, cCount,
                          out int colorCount,
                          out var cgOff, out var cgIdx);

            // ---- job chain ----
            JobHandle h = default;

            if (cCount > 0)
            {
                h = new PenaltyFloorJob
                {
                    bodies = bodyData, contacts = contactData,
                    penaltyMin = PenaltyMin, penaltyScale = PenaltyScale,
                    invDt2 = invDt2
                }.Schedule(cCount, 64, h);

                h = new ComputeC0Job
                {
                    bodies = bodyData, contacts = contactData
                }.Schedule(cCount, 64, h);
            }

            h = new InitBodiesJob
            {
                bodies = bodyData, bcOffsets = bcOff,
                gravity = gravity, dt = dt
            }.Schedule(bCount, 64, h);

            for (int iter = 0; iter < iterations; iter++)
            {
                if (cCount > 0)
                {
                    h = new ComputeConstraintsJob
                    {
                        bodies = bodyData, contacts = contactData, alpha = alpha
                    }.Schedule(cCount, 64, h);
                }

                for (int c = 0; c < colorCount; c++)
                {
                    int off = cgOff[c];
                    int cnt = cgOff[c + 1] - off;
                    if (cnt <= 0) continue;
                    h = new PrimalUpdateJob
                    {
                        bodyIndices = cgIdx.GetSubArray(off, cnt),
                        bodies      = bodyData,
                        contacts    = contactData,
                        bcOffsets   = bcOff,
                        bcIndices   = bcIdx,
                        invDt2      = invDt2,
                        alpha       = alpha
                    }.Schedule(cnt, 1, h);
                }

                if (cCount > 0)
                {
                    h = new DualUpdateJob
                    {
                        bodies = bodyData, contacts = contactData,
                        alpha = alpha, beta = beta, penaltyMax = PenaltyMax
                    }.Schedule(cCount, 64, h);
                }
            }

            h = new VelocityUpdateJob
            {
                bodies = bodyData, invDt = invDt
            }.Schedule(bCount, 64, h);

            h.Complete();

            CopyNativeToBodies(bodyData);
            SaveDualCacheFromNative(contactData, bodyData, cCount);

            bodyData.Dispose();
            contactData.Dispose();
            bcOff.Dispose();
            bcIdx.Dispose();
            cgOff.Dispose();
            cgIdx.Dispose();
        }

        // -------------------- data copy --------------------

        void CopyBodiesToNative(NativeArray<AvbdBodyData> dst)
        {
            for (int i = 0; i < bodies.Count; i++)
            {
                var b = bodies[i];
                dst[i] = new AvbdBodyData
                {
                    position        = b.position,
                    rotation        = b.rotation,
                    velocity        = b.velocity,
                    angularVelocity = b.angularVelocity,
                    prevVelocity    = b.prevVelocity,
                    initialPosition = b.initialPosition,
                    initialRotation = b.initialRotation,
                    inertialPosition = b.inertialPosition,
                    inertialRotation = b.inertialRotation,
                    mass            = b.isStatic ? 0f : b.mass,
                    halfExtents     = b.GetHalfExtents()
                };
            }
        }

        void CopyNativeToBodies(NativeArray<AvbdBodyData> src)
        {
            for (int i = 0; i < bodies.Count; i++)
            {
                var bd = src[i];
                if (bd.mass <= 0) continue;
                var b = bodies[i];
                b.position        = bd.position;
                b.rotation        = bd.rotation;
                b.velocity        = bd.velocity;
                b.angularVelocity = bd.angularVelocity;
                b.prevVelocity    = bd.prevVelocity;
                b.initialPosition = bd.initialPosition;
                b.initialRotation = bd.initialRotation;
                b.inertialPosition = bd.inertialPosition;
                b.inertialRotation = bd.inertialRotation;
                b.transform.position = bd.position;
                b.transform.rotation = bd.rotation;
            }
        }

        // -------------------- body-contact map --------------------

        static void BuildBodyContactMap(
            int bodyCount, NativeArray<AvbdContact> cd, int cCount,
            out NativeArray<int> offsets, out NativeArray<int> indices)
        {
            int[] counts = new int[bodyCount];
            for (int ci = 0; ci < cCount; ci++)
            {
                var c = cd[ci];
                if (c.bodyA >= 0 && c.bodyA < bodyCount) counts[c.bodyA]++;
                if (c.bodyB >= 0 && c.bodyB < bodyCount) counts[c.bodyB]++;
            }

            offsets = new NativeArray<int>(bodyCount + 1, Allocator.TempJob);
            offsets[0] = 0;
            for (int i = 0; i < bodyCount; i++) offsets[i + 1] = offsets[i] + counts[i];

            int total = offsets[bodyCount];
            indices = new NativeArray<int>(math.max(total, 1), Allocator.TempJob);

            int[] cur = new int[bodyCount];
            for (int ci = 0; ci < cCount; ci++)
            {
                var c = cd[ci];
                if (c.bodyA >= 0 && c.bodyA < bodyCount)
                    indices[offsets[c.bodyA] + cur[c.bodyA]++] = ci;
                if (c.bodyB >= 0 && c.bodyB < bodyCount)
                    indices[offsets[c.bodyB] + cur[c.bodyB]++] = ci;
            }
        }

        // -------------------- graph coloring --------------------

        static void BuildColoring(
            int bodyCount, NativeArray<AvbdContact> cd, int cCount,
            out int colorCount,
            out NativeArray<int> cgOffsets,
            out NativeArray<int> cgIndices)
        {
            var adj = new List<HashSet<int>>(bodyCount);
            for (int i = 0; i < bodyCount; i++) adj.Add(new HashSet<int>());
            var dynSet = new HashSet<int>();

            for (int ci = 0; ci < cCount; ci++)
            {
                var c = cd[ci];
                if (c.bodyA >= 0) dynSet.Add(c.bodyA);
                if (c.bodyB >= 0 && c.bodyB < bodyCount)
                {
                    dynSet.Add(c.bodyB);
                    adj[c.bodyA].Add(c.bodyB);
                    adj[c.bodyB].Add(c.bodyA);
                }
            }

            int[] colors = new int[bodyCount];
            for (int i = 0; i < bodyCount; i++) colors[i] = -1;

            var sorted = new List<int>(dynSet);
            sorted.Sort((a, b) => adj[b].Count.CompareTo(adj[a].Count));

            int maxC = -1;
            var used = new HashSet<int>();
            foreach (int bi in sorted)
            {
                used.Clear();
                foreach (int nb in adj[bi])
                    if (colors[nb] >= 0) used.Add(colors[nb]);
                int col = 0;
                while (used.Contains(col)) col++;
                colors[bi] = col;
                if (col > maxC) maxC = col;
            }

            colorCount = maxC + 1;
            if (colorCount <= 0)
            {
                cgOffsets = new NativeArray<int>(1, Allocator.TempJob);
                cgOffsets[0] = 0;
                cgIndices = new NativeArray<int>(1, Allocator.TempJob);
                return;
            }

            var groups = new List<List<int>>(colorCount);
            for (int c = 0; c < colorCount; c++) groups.Add(new List<int>());
            for (int i = 0; i < bodyCount; i++)
                if (colors[i] >= 0) groups[colors[i]].Add(i);

            int totalIdx = 0;
            foreach (var g in groups) totalIdx += g.Count;

            cgOffsets = new NativeArray<int>(colorCount + 1, Allocator.TempJob);
            cgIndices = new NativeArray<int>(math.max(totalIdx, 1), Allocator.TempJob);
            int off = 0;
            for (int c = 0; c < colorCount; c++)
            {
                cgOffsets[c] = off;
                foreach (int bi in groups[c]) cgIndices[off++] = bi;
            }
            cgOffsets[colorCount] = off;
        }

        // -------------------- dual cache --------------------

        void SaveDualCacheFromNative(
            NativeArray<AvbdContact> cd, NativeArray<AvbdBodyData> bd, int cCount)
        {
            dualCache.Clear();
            for (int ci = 0; ci < cCount; ci++)
            {
                var c = cd[ci];
                var bA = bd[c.bodyA];
                float3 wA = bA.position + math.rotate(bA.rotation, c.rA);
                long key = MakeKey(c.bodyA, c.bodyB, wA, c.normal);
                dualCache[key] = new CachedDual { lambda = c.lambda, penalty = c.penalty };
            }
        }

        static long MakeKey(int bA, int bB, float3 pos, float3 n)
        {
            const float Q = 50f;
            int h = bA * 73856093 ^ (bB + 1) * 19349663;
            h ^= (int)math.round(pos.x * Q) * 83492791;
            h ^= (int)math.round(pos.y * Q) * 14097643;
            h ^= (int)math.round(pos.z * Q) * 57395761;
            h ^= (int)math.round(n.x * Q) * 37656571;
            h ^= (int)math.round(n.y * Q) * 26791023;
            h ^= (int)math.round(n.z * Q) * 48397187;
            return h;
        }
    }
}
