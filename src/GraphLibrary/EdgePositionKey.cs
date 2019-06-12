namespace GraphLibrary
{
    public struct EdgePositionKey
    {
        public readonly int X;
        public readonly int Y;

        public EdgePositionKey(int x, int y)
        {
            X = x;
            Y = y;
        }

        public override int GetHashCode()
        {
            return X ^ Y;
        }

        public override bool Equals(object obj)
        {
            if (!(obj is EdgePositionKey)) return false;
            var that = (EdgePositionKey)obj;
            return X == that.X && Y == that.Y;
        }
    }
}
