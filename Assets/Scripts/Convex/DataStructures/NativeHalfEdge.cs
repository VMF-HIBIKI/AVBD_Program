namespace Convex.DataStructures
{
    public struct NativeHalfEdge
    {
        public int StartVertex;//起始点
        public int BelongFace;//所属面索引
        public int NextHalfedge;//下一条半边
        public int PrevHalfedge;//上一条半边
        public int TwinHalfedge;//反向边
    }
}