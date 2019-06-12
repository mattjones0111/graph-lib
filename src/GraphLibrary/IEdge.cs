namespace GraphLibrary
{
    public interface IEdge<out T>
    {
        /// <summary>
        /// The vertex at the start of the out-edge.
        /// </summary>
        T Tail { get; }

        /// <summary>
        /// The vertex at the end of the out-edge.
        /// </summary>
        T Head { get; }

        /// <summary>
        /// The cost of the moving from tail to head.
        /// </summary>
        int Weight { get; }
    }
}
