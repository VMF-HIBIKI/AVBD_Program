using Unity.Mathematics;

namespace Convex.DataStructures
{
    public struct NativeVertex
    {
        private float3 _position;
        
        public float3 Position
        {
            get { return _position;}
            set { _position = value; }
        }
    }
}