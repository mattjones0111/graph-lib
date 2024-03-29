﻿using System;
using System.Collections.Generic;
using System.Linq;

namespace GraphLibrary
{
    public class Graph<TEdge, TVertex> where TEdge : class, IEdge<TVertex>
    {
        public Graph(TEdge[] edges)
        {
            if (edges == null)
            {
                throw new ArgumentNullException();
            }

            if (edges.Length == 0)
            {
                throw new ArgumentException();
            }

            // verify that each member of the edges enumerable is non-null
            if (edges.Any(x => x == null))
            {
                throw new ArgumentException(
                    "Edges parameter contains null members.");
            }

            // verify that each edge has non-null heads and tails
            if (edges.Any(x => x.Head == null) ||
                edges.Any(x => x.Tail == null))
            {
                throw new ArgumentException(
                    "Edges parameter contains heads or tails that are null.");
            }

            Edges = edges;

            // build arrays of the edges, and the distinct vertices
            Vertices = edges
                .Select(x => x.Tail)
                .Concat(edges.Select(x => x.Head))
                .Distinct()
                .ToArray();

            // build three arrays of heads, tails and weights
            TailVertices = new[] { 0 }
                .Concat(Edges.Select(x => Array.IndexOf(Vertices, x.Tail) + 1))
                .ToArray();

            HeadVertices = new[] { 0 }
                .Concat(Edges.Select(x => Array.IndexOf(Vertices, x.Head) + 1))
                .ToArray();

            Weights = new[] { 0 }
                .Concat(Edges.Select(x => x.Weight))
                .ToArray();

            EdgePositionDictionary = Edges
                .ToDictionary(x =>
                    new EdgePositionKey(
                        Array.IndexOf(Vertices, x.Tail),
                        Array.IndexOf(Vertices, x.Head)));
        }

        Dictionary<EdgePositionKey, TEdge> EdgePositionDictionary
        {
            get;
        }

        public TEdge[] Edges
        {
            get;
        }

        public TVertex[] Vertices
        {
            get;
        }

        public int[] TailVertices
        {
            get;
        }

        public int[] HeadVertices
        {
            get;
        }

        public int[] Weights
        {
            get;
        }

        /// <summary>
        ///    Builds a path (represented by a collection of edges) from
        ///    the found path integer array.
        /// </summary>
        /// <typeparam name="TEdge"></typeparam>
        /// <typeparam name="TVertex"></typeparam>
        /// <param name="graph"></param>
        /// <param name="resultPathArray"></param>
        /// <param name="pathNumber"></param>
        /// <returns></returns>
        public TEdge[] BuildResultFromPath(int[,] resultPathArray, int pathNumber)
        {
            var numberOfVerticesVisited = resultPathArray[pathNumber, 0];
            var result = new TEdge[numberOfVerticesVisited - 1];

            for (var i = 2; i <= numberOfVerticesVisited; i++)
            {
                var tailPosition = resultPathArray[pathNumber, i - 1] - 1;
                var headPosition = resultPathArray[pathNumber, i] - 1;

                result[i-2] = EdgePositionDictionary[new EdgePositionKey(tailPosition, headPosition)];
            }

            return result;
        }
    }
}
