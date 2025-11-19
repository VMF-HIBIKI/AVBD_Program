using Unity.Mathematics;

namespace AVBD
{
    public class FContact
    {
        public float3 ContactLocalPointA;
        public float3 ContactLocalPointB;
        
        public float3 ContactWorldSpacePositionA;
        public float3 ContactWorldSpacePositionB;

        public float3 JAn, JAt, JAb;
        public float3 JWn, JWt, JWb;
        public float3 C0;
        public bool Stick;
        
        public void Initialize(float3 InHitNormal, float3 InContactAObjectPosition, float3 InContactWorldSpacePositionA,float3 InContactWorldSpacePositionB)
        {
            ContactWorldSpacePositionA = InContactWorldSpacePositionA;
            ContactWorldSpacePositionB = InContactWorldSpacePositionB;
            ContactLocalPointA = InContactWorldSpacePositionA - InContactAObjectPosition;

            float3 Normal = InHitNormal;
            float3 a = math.abs(Normal.x) > 0.9 ? new float3(0, 1, 0) : new float3(1, 0, 0);
            float3 Tangent = (math.cross(Normal, a)); //{Normal.x, Normal.z, Normal.y};
            float3 Binormal = (math.cross(Tangent, Normal));
            JAn = Normal;
            JAt = Tangent;
            JAb = Binormal;
            float3x3 Basis;
            Basis.c0 = JAn;
            Basis.c1 = JAt;
            Basis.c2 = JAb;
            
            //方便将 “旋转角速度 ω” 转换为 “接触点在法线方向上的速度贡献” 的桥梁
            JWn = math.cross(ContactLocalPointA, Normal);
            JWt = math.cross(ContactLocalPointA, Tangent);
            JWb = math.cross(ContactLocalPointA, Binormal);
            /*
            JWn = math.cross(Normal,ContactLocalPointA);
            JWt = math.cross(Tangent,ContactLocalPointA);
            JWb = math.cross(Binormal,ContactLocalPointA);*/
            
            //三个维度的position
            C0 = math.mul(math.transpose(Basis), InContactWorldSpacePositionA - InContactWorldSpacePositionB);
        }
    }

    public class Force
    {
        public FContact Contact;
        public float3x3 JA;
        public float3x3 JW;
        public float3 C;
        public float3 K;
        public float3 Lambda;
        public float Friction;
        public float3 Fmin;
        public float3 Fmax;
        public int otherBodyId;
        
        
        public void Initial(FContact InContact, float3 InPenalty, float InFriction, float3 InLambda)
        {
            Contact = InContact;
            K = InPenalty;
            Friction = InFriction;
            Lambda = InLambda;

            JA.c0 = Contact.JAn;
            JA.c1 = Contact.JAt;
            JA.c2 = Contact.JAb;
            JW.c0 = Contact.JWn; //for rotation
            JW.c1 = Contact.JWt;
            JW.c2 = Contact.JWb;
        }
        
        public void ComputeConstraint(float Alpha, float3 DeltaPositionA, float3 DeltaWA)
        {

            float DotContactAN = math.dot(Contact.JAn, DeltaPositionA);
            float DotContactAT = math.dot(Contact.JAt, DeltaPositionA);
            float DotContactAB = math.dot(Contact.JAb, DeltaPositionA);

            float DotContactANR = math.dot(Contact.JWn, DeltaWA);
            float DotContactATR = math.dot(Contact.JWt, DeltaWA);
            float DotContactABR = math.dot(Contact.JWb, DeltaWA);
            C = Contact.C0 * (1 - Alpha) + new float3(DotContactAN, DotContactAT, DotContactAB) + new float3(DotContactANR, DotContactATR, DotContactABR);
            //Contact.C0 = C;
            float FrictionBound = math.abs(Lambda.x) * Friction;
            Fmin = new float3(-1000000, -FrictionBound, -FrictionBound);
            Fmax = new float3(0, FrictionBound, FrictionBound);
            // Check if the contact is sticking, so that on the next frame we can use the old contact points for better static friction handling
            Contact.Stick = math.abs(Lambda.y) < FrictionBound && math.abs(Contact.C0.y) < 0.02f;
        }
    }
}