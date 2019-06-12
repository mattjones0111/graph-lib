namespace GraphLibrary
{
    public interface IFindShortestPaths
    {
        TEdge[][] FindShortestPaths<TEdge, TVertex>(
            Graph<TEdge, TVertex> graph,
            TVertex origin,
            TVertex destination,
            int numberOfPaths)
            where TEdge : class, IEdge<TVertex>;

        /// <summary>
        /// Overloaded. Allows the injection of an overridden
        /// array of weights.
        /// </summary>
        /// <typeparam name="TEdge"></typeparam>
        /// <typeparam name="TVertex"></typeparam>
        /// <param name="graph"></param>
        /// <param name="overrideWeights"></param>
        /// <param name="origin"></param>
        /// <param name="destination"></param>
        /// <param name="numberOfPaths"></param>
        /// <returns></returns>
        TEdge[][] FindShortestPaths<TEdge, TVertex>(
            Graph<TEdge, TVertex> graph,
            int[] overrideWeights,
            TVertex origin,
            TVertex destination,
            int numberOfPaths)
            where TEdge : class, IEdge<TVertex>;
    }
}
