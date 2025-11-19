using Unity.Mathematics;

public static class MatrixUtils
{
    /// <summary>
    /// 对称正定矩阵 A（float3x3）用 LDLᵀ 分解并求逆。
    /// A = L * D * Lᵀ
    /// 返回 A⁻¹。
    /// </summary>
    public static float3x3 InverseLDL(float3x3 A)
    {
        // 假设 A 是对称正定矩阵（SPD）
        float3x3 L = float3x3.identity;
        float3 D = float3.zero; // 只存储对角 D 元素

        // 提取矩阵分量
        float a11 = A.c0.x;
        float a21 = A.c0.y; float a22 = A.c1.y;
        float a31 = A.c0.z; float a32 = A.c1.z; float a33 = A.c2.z;

        // --- LDL 分解 ---
        // 第一行
        D.x = a11;

        // 第二行
        L.c0.y = a21 / D.x;
        D.y = a22 - L.c0.y * L.c0.y * D.x;

        // 第三行
        L.c0.z = a31 / D.x;
        L.c1.z = (a32 - L.c0.z * L.c0.y * D.x) / D.y;
        D.z = a33 - (L.c0.z * L.c0.z * D.x + L.c1.z * L.c1.z * D.y);

        // --- 计算逆矩阵 A⁻¹ = (L * D * Lᵀ)⁻¹ = L⁻ᵀ * D⁻¹ * L⁻¹ ---

        // 先求 L⁻¹（L 是下三角）
        float3x3 Linv = float3x3.identity;
        Linv.c0.y = -L.c0.y;
        Linv.c0.z = -(L.c0.z - L.c1.z * L.c0.y);
        Linv.c1.z = -L.c1.z;

        // 再求 D⁻¹
        float3x3 Dinv = float3x3.zero;
        Dinv.c0.x = 1.0f / (D.x + 1e-8f);
        Dinv.c1.y = 1.0f / (D.y + 1e-8f);
        Dinv.c2.z = 1.0f / (D.z + 1e-8f);

        // A⁻¹ = L⁻ᵀ * D⁻¹ * L⁻¹
        float3x3 temp = math.mul(Dinv, Linv);
        float3x3 Ainv = math.mul(math.transpose(Linv), temp);

        return Ainv;
    }
    
    public static void PolarDecomposition(ref float3x3 A, out float3x3 R)
    {
        // 初始矩阵
        R = A;

        // Newton–Schulz iteration
        for (int iter = 0; iter < 5; iter++) // 迭代 3~5 次通常足够
        {
            float3x3 RinvT = math.transpose(InverseFast(R));
            R = 0.5f * (R + RinvT);
        }

        // 归一化，避免数值漂移
        Orthonormalize(ref R);
    }

    
    private static float3x3 InverseFast(float3x3 m)
    {
        float3 a = m.c0;
        float3 b = m.c1;
        float3 c = m.c2;

        float3 r0 = math.cross(b, c);
        float3 r1 = math.cross(c, a);
        float3 r2 = math.cross(a, b);

        float det = math.dot(r2, c);
        if (math.abs(det) < 1e-8f)
            return float3x3.identity;

        float invDet = 1.0f / det;
        return new float3x3(r0 * invDet, r1 * invDet, r2 * invDet);
    }
    
    private static void Orthonormalize(ref float3x3 R)
    {
        float3 x = R.c0;
        float3 y = R.c1;
        float3 z = R.c2;

        x = math.normalize(x);
        y = math.normalize(y - x * math.dot(x, y));
        z = math.cross(x, y);

        R.c0 = x;
        R.c1 = y;
        R.c2 = z;
    }

    public static float3 Matrix3X3_float3(float3x3 matrix, float3 vector)
    {
        // 结果向量的x分量 = 矩阵第1行 · 输入向量
        float x = math.dot(matrix.c0.xyz, vector);  // matrix.c0 是矩阵第1行（x行）
        // 结果向量的y分量 = 矩阵第2行 · 输入向量
        float y = math.dot(matrix.c1.xyz, vector);  // matrix.c1 是矩阵第2行（y行）
        // 结果向量的z分量 = 矩阵第3行 · 输入向量
        float z = math.dot(matrix.c2.xyz, vector);  // matrix.c2 是矩阵第3行（z行）

        return new float3(x, y, z);
    }
    
    public static quaternion UpdateRotation(quaternion currentRot, float3 angVel, float dt)
    {
        // 1. 计算旋转增量（微小旋转的四元数表示）
        // 公式：增量四元数 = [0, (ω × dt) / 2]（纯四元数，实部为0，虚部为角速度×时间的一半）
        float3 deltaAng = angVel * dt * 0.5f; // 角速度积分的一半（小角度近似）
        quaternion deltaRot = new quaternion(0, deltaAng.x, deltaAng.y, deltaAng.z); // 纯四元数

        // 2. 归一化增量四元数（确保是单位四元数，避免缩放）
        deltaRot = math.normalize(deltaRot);

        // 3. 组合旋转：当前旋转 × 增量旋转（四元数乘法用math.mul）
        // 注意顺序：先应用增量旋转，再叠加到当前旋转上
        quaternion newRot = math.mul(currentRot, deltaRot);

        // 4. 归一化结果（消除浮点误差导致的非单位性）
        return math.normalize(newRot);
    }
    
    public static float3x3 OuterProduct(float3 a, float3 b)
    {
        // 第一行：a.x 分别与 b的三个分量相乘
        float3 row0 = new float3(a.x * b.x, a.x * b.y, a.x * b.z);
        // 第二行：a.y 分别与 b的三个分量相乘
        float3 row1 = new float3(a.y * b.x, a.y * b.y, a.y * b.z);
        // 第三行：a.z 分别与 b的三个分量相乘
        float3 row2 = new float3(a.z * b.x, a.z * b.y, a.z * b.z);

        // 用三行构造3x3矩阵（float3x3的构造函数接受三行float3）
        return new float3x3(row0, row1, row2);
    }
}