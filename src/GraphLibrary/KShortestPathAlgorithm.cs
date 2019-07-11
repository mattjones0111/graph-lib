using System;
using System.Linq;

namespace GraphLibrary
{
    public class KShortestPathAlgorithm : IFindShortestPaths
    {
        #region Public methods

        /// <summary>
        ///     Returns a collection of edges that represent the ranked
        ///     shortest paths between origin and destination vertices,
        ///     bounded by numberOfPaths value.
        /// </summary>
        /// <typeparam name="TEdge">The type of a graph edge.</typeparam>
        /// <typeparam name="TVertex">The type of a graph vertex.</typeparam>
        /// <param name="graph">The graph.</param>
        /// <param name="origin">The origin vertex.</param>
        /// <param name="destination">The destination vertex.</param>
        /// <param name="numberOfPaths">The maximum number of paths to return.</param>
        /// <returns>
        ///     A collection of TEdge arrays that represent the ranked
        ///     shortest routes through the graph.
        /// </returns>
        public TEdge[][] FindShortestPaths<TEdge, TVertex>(
            Graph<TEdge, TVertex> graph,
            TVertex origin,
            TVertex destination,
            int numberOfPaths)
            where TEdge : class, IEdge<TVertex>
        {
            if (graph == null)
            {
                throw new ArgumentNullException();
            }

            return FindShortestPaths(
                graph,
                graph.Weights,
                origin,
                destination,
                numberOfPaths);
        }

        /// <summary>
        ///     Returns a collection of edges that represent the ranked
        ///     shortest paths between origin and destination vertices,
        ///     bounded by numberOfPaths value. Allows the weights to be
        ///     overridden.
        /// </summary>
        /// <typeparam name="TEdge">The type of a graph edge.</typeparam>
        /// <typeparam name="TVertex">The type of a graph vertex.</typeparam>
        /// <param name="graph">The graph.</param>
        /// <param name="overrideWeights">Weights to use instead of those defined by the graph.</param>
        /// <param name="origin">The origin vertex.</param>
        /// <param name="destination">The destination vertex.</param>
        /// <param name="numberOfPaths">The maximum number of paths to return.</param>
        /// <returns>
        ///     A collection of TEdge arrays that represent the ranked
        ///     shortest routes through the graph.
        /// </returns>
        public TEdge[][] FindShortestPaths<TEdge, TVertex>(
            Graph<TEdge, TVertex> graph,
            int[] overrideWeights,
            TVertex origin,
            TVertex destination,
            int numberOfPaths)
            where TEdge : class, IEdge<TVertex>
        {
            if (graph == null || origin == null ||
                destination == null || overrideWeights == null)
                throw new ArgumentNullException();

            if (overrideWeights.Length != graph.Edges.Length + 1)
                throw new ArgumentException(
                    "Weights must contain one fewer members than graph edges.");

            if (!graph.Vertices.Contains(origin) ||
                !graph.Vertices.Contains(destination))
                throw new ArgumentException(
                    "Origin or destination not part of input graph.");

            var resultPathArray = new int[
                numberOfPaths + 1,
                graph.Vertices.Length + 2];

            FindShortestPaths(graph.Vertices.Length,
                graph.Edges.Count(),
                graph.TailVertices,
                graph.HeadVertices,
                overrideWeights,
                numberOfPaths,
                Array.IndexOf(graph.Vertices, origin) + 1,
                Array.IndexOf(graph.Vertices, destination) + 1,
                ref resultPathArray);

            var numberOfReturnPaths = resultPathArray[0, 0];

            // no paths found? return an empty array of edges
            if (numberOfReturnPaths == 0)
                return new TEdge[][] { };

            var result = new TEdge[numberOfReturnPaths][];

            for (var i = 1; i <= numberOfReturnPaths; i++)
            {
                result[i-1] = graph.BuildResultFromPath(resultPathArray, i);
            }

            return result;
        }

        #endregion

        #region Low level k-shortest path finding method

        /// <summary>
        ///     Low-level K-shortest path algorithm.
        /// </summary>
        /// <param name="n">Number of unique nodes</param>
        /// <param name="m">Number of edges</param>
        /// <param name="nodei">Tail nodes</param>
        /// <param name="nodej">Head nodes</param>
        /// <param name="weight">Edge weights</param>
        /// <param name="k">Number of paths to return</param>
        /// <param name="source">Origin node</param>
        /// <param name="sink">Destination node</param>
        /// <param name="path">Output parameter. The result.</param>
        void FindShortestPaths(int n, int m, int[] nodei, int[] nodej, int[] weight,
            int k, int source, int sink, ref int[,] path)
        {
            // safeguards
            if (sink <= 0)
                throw new Exception("Sink cannot be less than 1. Sink = " + sink);

            // Checking the mapping in nodei and nodej arrays are correct
            if (nodei[0] != 0)
                throw new Exception("Nodei[0] is non-zero");

            if (nodej[0] != 0)
                throw new Exception("Nodej[0] is non-zero");

            if (weight[0] != 0)
                throw new Exception("Weight[0] is non-zero");

            int i, j, jj, kk, large, tail, head, node1, node2, kp3, length;
            int incrs3, index1, index2, index3, index4, poolc;
            int j1, j2, j3, quefirstnode, linkst, treenode, examine, edge1, edge2;
            int bedge, cedge, dedge, jem1, jem2, njem1, njem2, njem3, numpaths;
            int upbound, lenrst, detb, deta, nrda, nrdb, quelast;
            int order, nrdsize, hda, hdb, count, queorder, quefirst, quenext;
            int pos1, pos2, pos3, auxda, auxdb, auxdc, low, high;
            int number, nump, sub1, sub2, sub3, jsub, nextnode;
            int nodep1 = 0, nodep2 = 0, nodep3 = 0, ncrs1 = 0, ncrs2 = 0, ncrs3 = 0;
            int edge3 = 0, lenga = 0, lengb = 0, mark1 = 0, mark2 = 0, mshade = 0;
            int jump = 0, incrs2 = 0, incrs4 = 0, jnd1 = 0, jnd2 = 0, jterm = 0, jedge = 0;
            int lentab = 0, nqop1 = 0, nqop2 = 0, nqsize = 0, nqin = 0, parm = 0;
            int nqout1 = 0, nqout2 = 0, nqp1 = 0, nqp2 = 0, poola = 0, poolb = 0;
            int incrs1 = 0, nqfirst = 0, nqlast = 0;

            int[] shortpathtree = new int[n + 1];
            int[] treedist = new int[n + 1];
            int[] arcnode = new int[n + 1];
            int[] arcforward = new int[n + 1];
            int[] arcbackward = new int[n + 1];
            int[] auxstore = new int[n + 1];
            int[] auxdist = new int[n + 1];
            int[] auxtree = new int[n + 1];
            int[] auxlink = new int[n + 1];

            int[] nextforward = new int[m + 1];
            int[] nextbackward = new int[m + 1];

            int[] queuefirstpath = new int[k + 4];
            int[] queuenextpath = new int[k + 4];
            int[] queuesearch = new int[k + 4];
            int[] kpathlength = new int[k + 4];

            int maxqueuelength = 10 * k;
            int[] crossarc = new int[maxqueuelength + 1];
            int[] nextpoolentry = new int[maxqueuelength + 1];

            bool forwrd, lastno, noroom, goon, resultfound, getpaths;
            bool loopon, lasta, lastb, rdfull, skip, force;
            bool finem1 = false, finem2 = false, initsp = false, nostg = false;
            bool invoke = false;

            kp3 = k + 3;

            # region Set up and initialise the network

            for (i = 1; i <= n; i++)
            {
                arcforward[i] = 0;
                arcbackward[i] = 0;
            }

            large = 1;
            for (i = 1; i <= m; i++)
            {
                large += weight[i];
                tail = nodei[i];
                head = nodej[i];
                nextforward[i] = arcforward[tail];
                arcforward[tail] = i;
                nextbackward[i] = arcbackward[head];
                arcbackward[head] = i;
            }

            //Initialisation
            for (i = 1; i <= n; i++)
                auxdist[i] = large;
            for (i = 1; i <= kp3; i++)
                queuefirstpath[i] = 0;
            for (i = 1; i <= maxqueuelength; i++)
                nextpoolentry[i] = i;
            nextpoolentry[maxqueuelength] = 0;

            # endregion

            # region Build the shortest distance tree

            //Treedist[i] is used to store the shortest distance of node i from source
            //shortpathtree[i] will contain the tree arc coming to node i,
            //it is negative when the direction of the arc is towards source.
            // it is zero if it is note reachable

            for (i = 1; i <= n; i++)
            {
                treedist[i] = large;
                shortpathtree[i] = 0;
                arcnode[i] = 0;
            }

            treedist[source] = 0;
            shortpathtree[source] = source;
            arcnode[source] = source;
            j = source;
            node1 = source;

            // examine neighbours of node j
            do
            {
                edge1 = arcforward[j];
                forwrd = true;
                lastno = false;
                do
                {
                    if (edge1 == 0)
                        lastno = true;
                    else
                    {
                        length = treedist[j] + weight[edge1];
                        if (forwrd)
                        {
                            node2 = nodej[edge1];
                            edge2 = edge1;
                        }
                        else
                        {
                            node2 = nodei[edge1];
                            edge2 = -edge1;
                        }
                        if (length < treedist[node2])
                        {
                            treedist[node2] = length;
                            shortpathtree[node2] = edge2;
                            if (arcnode[node2] == 0)
                            {
                                arcnode[node1] = node2;
                                arcnode[node2] = node2;
                                node1 = node2;
                            }
                            else
                            {
                                if (arcnode[node2] < 0)
                                {
                                    arcnode[node2] = arcnode[j];
                                    arcnode[j] = node2;
                                    if (node1 == j)
                                    {
                                        node1 = node2;
                                        arcnode[node2] = node2;
                                    }
                                }
                            }
                        }
                        if (forwrd)
                            edge1 = nextforward[edge1];
                        else
                            edge1 = nextbackward[edge1];
                    }
                } while (!lastno);
                jj = j;
                j = arcnode[j];
                arcnode[jj] = -1;
            } while (j != jj);

            //Finish building the shortest distance tree
            numpaths = 0;
            resultfound = false;
            noroom = false;
            getpaths = false;

            if (shortpathtree[sink] == 0)
            {
                getpaths = true;
                resultfound = true;
            }

            # endregion

            # region Initialise the storage pool

            if (!getpaths)
            // Initialise the storage pool
            {
                i = 1;
                do
                {
                    queuenextpath[i] = i;
                    i++;
                } while (i <= k + 2);
                queuenextpath[k + 3] = 0;
                //initialise the priority queue
                lentab = kp3;
                low = -large;
                high = large;
                nqop1 = lentab;
                nqop2 = 0;
                nqsize = 0;
                nqin = 0;
                nqout1 = 0;
                nqout2 = 0;
                // obtain an entry from the pool
                index1 = queuenextpath[1];
                queuenextpath[1] = queuenextpath[index1 + 1];
                index2 = queuenextpath[1];
                queuenextpath[1] = queuenextpath[index2 + 1];
                kpathlength[index1 + 1] = low;
                queuenextpath[index1 + 1] = index2;
                kpathlength[index2 + 1] = high;
                queuenextpath[index2 + 1] = 0;
                nqp1 = 0;
                nqp2 = 1;
                queuesearch[1] = index1;
                queuesearch[2] = index2;
                nqfirst = high;
                nqlast = low;
                // Set the shortest path to the queue
                poola = queuenextpath[1];
                queuenextpath[1] = queuenextpath[poola + 1];
                poolb = poola;
                incrs1 = nextpoolentry[1];
                nextpoolentry[1] = nextpoolentry[incrs1 + 1];
                crossarc[incrs1 + 1] = shortpathtree[sink];
                nextpoolentry[incrs1 + 1] = 0;
                kpathlength[poola + 1] = treedist[sink];
                queuefirstpath[poola + 1] = incrs1;
                parm = poola;
                invoke = false;
            }

            # endregion

            # region Insert 'parm' into the priority queue

        iterate:
            while (true)
            {
                if (resultfound)
                    break;

                order = kpathlength[parm + 1];
                pos1 = nqp1;
                pos2 = nqp2;
                while (pos2 - pos1 > 1)
                {
                    pos3 = (pos1 + pos2) / 2;
                    if (order > kpathlength[queuesearch[pos3 + 1] + 1])
                        pos1 = pos3;
                    else
                        pos2 = pos3;
                }
                //linear search starting from queuesearch[pos1+1]
                index1 = queuesearch[pos1 + 1];
                do
                {
                    index2 = index1;
                    index1 = queuenextpath[index1 + 1];
                } while (kpathlength[index1 + 1] <= order);
                // insert between 'index1' and 'index2'
                queuenextpath[index2 + 1] = parm;
                queuenextpath[parm + 1] = index1;
                // update data in the queue
                nqsize = nqsize + 1;
                nqin = nqin + 1;
                nqop1 = nqop1 - 1;
                if (nqsize == 1)
                {
                    nqfirst = order;
                    nqlast = order;
                }
                else
                {
                    if (order > nqlast)
                        nqlast = order;
                    else
                        if (order < nqfirst)
                            nqfirst = order;
                }
                if (nqop1 <= 0)
                // reorganise
                {
                    index1 = queuesearch[nqp1 + 1];
                    queuesearch[1] = index1;
                    nqp1 = 0;
                    index2 = queuesearch[nqp2 + 1];
                    j3 = nqsize / lentab;
                    j2 = j3 + 1;
                    j1 = nqsize - ((nqsize / lentab) * lentab);

                    if (j1 > 0)
                        for (pos2 = 1; pos2 <= j1; pos2++)
                        {
                            for (i = 1; i <= j2; i++)
                                index1 = queuenextpath[index1 + 1];
                            queuesearch[pos2 + 1] = index1;
                        }

                    if (j3 > 0)
                    {
                        pos2 = j1 + 1;
                        while (pos2 <= lentab - 1)
                        {
                            for (i = 1; i <= j3; i++)
                                index1 = queuenextpath[index1 + 1];
                            queuesearch[pos2 + 1] = index1;
                            pos2++;
                        }
                    }
                    nqp2 = pos2;
                    queuesearch[nqp2 + 1] = index2;
                    nqop2 = nqop2 + 1;
                    nqop1 = nqsize / 2;
                    if (nqop1 < lentab)
                        nqop1 = lentab;
                }
                force = false;
                if (invoke)
                {
                    if (nostg)
                    {
                        resultfound = true;
                        goto iterate;
                    }
                    force = true;
                }
                if (!force)
                {
                    lenga = 0;
                    mark1 = 0;
                    initsp = true;
                    for (i = 1; i <= n; i++)
                        arcnode[i] = 0;
                }

                # region Process next path

                while (true)
                {
                    if (!force)
                    {
                        mark1 = mark1 + 2;
                        mark2 = mark1;
                        mshade = mark1 + 1;
                        // obtain the first entry from the priority queue
                        if (nqsize > 0)
                        {
                            index2 = queuesearch[nqp1 + 1];
                            index1 = queuenextpath[index2 + 1];
                            queuenextpath[index2 + 1] = queuenextpath[index1 + 1];
                            nqfirst = kpathlength[queuenextpath[index1 + 1] + 1];
                            if (index1 == queuesearch[nqp1 + 2])
                            {
                                nqp1++;
                                queuesearch[nqp1 + 1] = index2;
                            }
                            nqop1--;
                            nqsize--;
                            nqout1++;
                            poolc = index1;
                        }
                        else
                            poolc = 0;

                        if (poolc == 0)
                        {
                            // no more paths in queue, stop
                            noroom = noroom && (numpaths < k);
                            resultfound = true;
                            goto iterate;
                        }
                        queuenextpath[poolb + 1] = poolc;
                        poolb = poolc;
                        numpaths++;
                        if (numpaths > k)
                        {
                            noroom = false;
                            numpaths--;
                            resultfound = true;
                            goto iterate;
                        }
                        lengb = kpathlength[poolc + 1];
                        quefirstnode = queuefirstpath[poolc + 1];
                        if (lengb < lenga)
                        {
                            resultfound = true;
                            goto iterate;
                        }
                        lenga = lengb;
                        //examine the tail of the arc
                        incrs2 = quefirstnode;
                        ncrs2 = source;
                        nodep1 = n + 1;
                        //obtain data of next path
                        jump = 1;
                    }
                    while (true)
                    {
                        if (!force)
                        // obtain data for the next path
                        {
                            if (incrs2 == 0)
                                linkst = 3;
                            else
                            {
                                ncrs3 = ncrs2;
                                incrs1 = incrs2;
                                j = Math.Abs(incrs1) + 1;
                                incrs3 = crossarc[j];
                                incrs2 = nextpoolentry[j];
                                if (incrs3 > 0)
                                {
                                    ncrs1 = nodei[incrs3];
                                    ncrs2 = nodej[incrs3];
                                    incrs4 = incrs3;
                                }
                                else
                                {
                                    ncrs1 = nodej[-incrs3];
                                    ncrs2 = nodei[-incrs3];
                                    incrs4 = -incrs3;
                                }
                                finem1 = incrs2 <= 0;
                                linkst = (ncrs2 == ncrs3) ? 2 : 1;
                            }
                            if (jump == 1)
                            {
                                lengb -= weight[incrs4];
                                nodep1--;
                                auxstore[nodep1] = incrs1;
                                while (ncrs1 != ncrs3)
                                {
                                    j = Math.Abs(shortpathtree[ncrs1]);
                                    lengb -= weight[j];
                                    ncrs1 = (shortpathtree[ncrs1] > 0) ? nodei[j] : nodej[j];
                                }
                                if (!finem1)
                                {
                                    jump = 1;
                                    continue;
                                }
                                // store the tail of the arc
                                nodep2 = nodep1;
                                finem2 = finem1;
                                // obtain data of next path
                                jump = 2;
                                continue;
                            }
                            if (jump == 2)
                            {
                                if (linkst == 2)
                                {
                                    nodep2--;
                                    auxstore[nodep2] = incrs4;
                                    weight[incrs4] += large;
                                    finem2 = finem1;
                                    // obtain data of the next path
                                    jump = 2;
                                    continue;
                                }
                                // close the arc on the shortest path
                                finem2 = finem2 && (linkst != 3);
                                if (finem2)
                                {
                                    edge3 = Math.Abs(shortpathtree[ncrs3]);
                                    nodep2--;
                                    auxstore[nodep2] = edge3;
                                    weight[edge3] += large;
                                }
                            }
                            if (jump == 3)
                            {
                                if (linkst == 1)
                                {
                                    arcnode[ncrs2] = mark2;
                                    while (ncrs1 != ncrs3)
                                    {
                                        arcnode[ncrs2] = mark2;
                                        if (shortpathtree[ncrs1] > 0)
                                            ncrs1 = nodei[shortpathtree[ncrs1]];
                                        else
                                            ncrs1 = nodej[-shortpathtree[ncrs1]];
                                    }
                                    jump = 3;
                                    continue;
                                }
                                if (linkst == 2)
                                {
                                    jump = 4;
                                    continue;
                                }
                            }
                            if (jump == 4)
                            {
                                if (linkst == 2)
                                {
                                    jump = 4;
                                    continue;
                                }
                            }
                            //mark more nodes
                            if (linkst != 3)
                            {
                                arcnode[ncrs2] = mark2;
                                while (ncrs1 != ncrs3)
                                {
                                    arcnode[ncrs1] = mark2;
                                    if (shortpathtree[ncrs1] > 0)
                                        ncrs1 = nodei[shortpathtree[ncrs1]];
                                    else
                                        ncrs1 = nodej[-shortpathtree[ncrs1]];
                                }
                                jump = 3;
                                continue;
                            }
                            // generate descendants of the tail of the arc
                            nodep3 = nodep1;
                            incrs1 = auxstore[nodep3];
                            jnd1 = crossarc[incrs1 + 1];
                            //obtain the first node of the arc traversing forward
                            jnd2 = (jnd1 < 0) ? nodei[-jnd1] : nodej[jnd1];
                        }
                        // process a section
                        do
                        {
                            if (!force)
                            {
                                nodep3++;
                                jterm = jnd2;
                                jedge = jnd1;
                                if (nodep3 > n)
                                    jnd2 = source;
                                else
                                {
                                    incrs2 = auxstore[nodep3];
                                    jnd1 = crossarc[incrs2 + 1];
                                    jnd2 = (-jnd1 > 0) ? nodei[-jnd1] : nodej[jnd1];
                                }
                            }
                            // process a node
                            do
                            {
                                if (!force)
                                {
                                    mark1 += 2;
                                    treenode = mark1;
                                    examine = mark1 + 1;
                                    edge3 = Math.Abs(jedge);
                                    weight[edge3] += large;
                                    if (initsp)
                                        initsp = (nqin < k);
                                    upbound = (initsp) ? large : nqlast;
                                    // obtain the restricted shortest path from source to jterm
                                    lenrst = upbound;
                                    bedge = 0;
                                    auxdist[jterm] = 0;
                                    auxtree[jterm] = 0;
                                    auxlink[jterm] = 0;
                                    jem1 = jterm;
                                    jem2 = jem1;
                                    // examine the next node
                                    do
                                    {
                                        njem1 = jem1;
                                        auxda = auxdist[njem1];
                                        jem1 = auxlink[njem1];
                                        arcnode[njem1] = treenode;
                                        if (auxda + treedist[njem1] + lengb >= lenrst)
                                            continue;
                                        goon = true;
                                        lasta = false;
                                        edge1 = arcbackward[njem1];
                                        // loop through the arcs from njem1
                                        do
                                        {
                                            if (edge1 == 0)
                                                lasta = true;
                                            else
                                            {
                                                //process the arc edge1
                                                auxdb = auxda + weight[edge1];
                                                if (goon)
                                                {
                                                    njem2 = nodei[edge1];
                                                    edge2 = edge1;
                                                    edge1 = nextbackward[edge1];
                                                }
                                                else
                                                {
                                                    njem2 = nodej[edge1];
                                                    edge2 = -edge1;
                                                    edge1 = nextforward[edge1];
                                                }
                                                if (arcnode[njem2] != mark2)
                                                {
                                                    auxdc = auxdb + lengb + treedist[njem2];
                                                    if (auxdc >= lenrst)
                                                        continue;
                                                    if (arcnode[njem2] < mark2)
                                                    {
                                                        if (shortpathtree[njem2] + edge2 == 0)
                                                            arcnode[njem2] = mshade;
                                                        else
                                                        {
                                                            // examine the status of the path
                                                            loopon = true;
                                                            njem3 = njem2;
                                                            while (loopon && (njem3 != source))
                                                            {
                                                                if (arcnode[njem3] < mark2)
                                                                {
                                                                    j = shortpathtree[njem3];
                                                                    njem3 = (j > 0) ? nodei[j] : nodej[-j];
                                                                }
                                                                else
                                                                    loopon = false;
                                                            }
                                                            if (loopon)
                                                            {
                                                                // better path found
                                                                lenrst = auxdc;
                                                                bedge = edge2;
                                                                continue;
                                                            }
                                                            else
                                                            {
                                                                njem3 = njem2;
                                                                lastb = false;
                                                                do
                                                                {
                                                                    if (arcnode[njem3] < mark2)
                                                                    {
                                                                        arcnode[njem3] = mshade;
                                                                        j = shortpathtree[njem3];
                                                                        njem3 = (j > 0) ? nodei[j] : nodej[-j];
                                                                    }
                                                                    else
                                                                        lastb = true;
                                                                } while (!lastb);
                                                            }
                                                        }
                                                    }
                                                    if ((arcnode[njem2] < treenode) || (auxdb < auxdist[njem2]))
                                                    {
                                                        // update node njem2
                                                        auxdist[njem2] = auxdb;
                                                        auxtree[njem2] = edge2;
                                                        if (arcnode[njem2] != examine)
                                                        {
                                                            arcnode[njem2] = examine;
                                                            if (jem1 == 0)
                                                            {
                                                                jem1 = njem2;
                                                                jem2 = njem2;
                                                                auxlink[njem2] = 0;
                                                            }
                                                            else
                                                            {
                                                                if (arcnode[njem2] == treenode)
                                                                {
                                                                    auxlink[njem2] = jem1;
                                                                    jem1 = njem2;
                                                                }
                                                                else
                                                                {
                                                                    auxlink[njem2] = 0;
                                                                    auxlink[jem2] = njem2;
                                                                    jem2 = njem2;
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        } while (!lasta);
                                    } while (jem1 > 0);
                                    arcnode[jterm] = mark2;
                                    // finish processing the restricted path
                                    if ((bedge != 0) && (lenrst < upbound))
                                    {
                                        detb = 0;
                                        cedge = bedge;
                                        do
                                        {
                                            dedge = (cedge > 0) ? nodej[cedge] : nodei[-cedge];
                                            if ((cedge != shortpathtree[dedge]) || (dedge == jterm))
                                            {
                                                detb++;
                                                auxstore[detb] = cedge;
                                            }
                                            cedge = auxtree[dedge];
                                        } while (cedge != 0);
                                        // restore the path data
                                        deta = detb;
                                        nrda = nextpoolentry[1];
                                        quelast = large;
                                        nostg = false;
                                        while ((deta > 0) && (nrda > 0))
                                        {
                                            deta--;
                                            nrda = nextpoolentry[nrda + 1];
                                        }
                                        rdfull = (!initsp) && (numpaths + nqsize >= k);
                                        skip = false;
                                        while (rdfull || (deta > 0))
                                        {
                                            //remove the last path from the queue
                                            quelast = nqlast;
                                            noroom = true;
                                            rdfull = false;
                                            // get the last entry from the priority queue
                                            if (nqsize > 0)
                                            {
                                                index4 = queuesearch[nqp2 + 1];
                                                index3 = queuesearch[nqp2];
                                                if (queuenextpath[index3 + 1] == index4)
                                                {
                                                    nqp2--;
                                                    queuesearch[nqp2 + 1] = index4;
                                                    index3 = queuesearch[nqp2];
                                                }
                                                index2 = index3;
                                                while (index3 != index4)
                                                {
                                                    index1 = index2;
                                                    index2 = index3;
                                                    index3 = queuenextpath[index3 + 1];
                                                }
                                                queuenextpath[index1 + 1] = index4;
                                                nqlast = kpathlength[index1 + 1];
                                                nqop1--;
                                                nrdsize = index2;
                                                nqsize--;
                                                nqout2++;
                                            }
                                            else
                                                nrdsize = 0;
                                            if (nrdsize == 0)
                                            {
                                                nostg = true;
                                                if (nostg)
                                                {
                                                    resultfound = true;
                                                    goto iterate;
                                                }
                                                skip = true;
                                                break;
                                            }
                                            nrda = queuefirstpath[nrdsize + 1];
                                            while (nrda > 0)
                                            {
                                                j = nrda + 1;
                                                deta--;
                                                nrdb = nrda;
                                                nrda = nextpoolentry[j];
                                                nextpoolentry[j] = nextpoolentry[1];
                                                nextpoolentry[1] = nrdb;
                                            }
                                            // put the entry nrdsize to pool
                                            queuenextpath[nrdsize + 1] = queuenextpath[1];
                                            queuenextpath[1] = nrdsize;
                                        }
                                        if (!skip)
                                        {
                                            //build the entries of crossarc and nextpoolentry
                                            if (lenrst >= quelast)
                                            {
                                                if (nostg)
                                                {
                                                    resultfound = true;
                                                    goto iterate;
                                                }
                                            }
                                            else
                                            {
                                                nrdb = -incrs1;
                                                deta = detb;
                                                while (deta > 0)
                                                {
                                                    nrda = nextpoolentry[1];
                                                    nextpoolentry[1] = nextpoolentry[nrda + 1];
                                                    crossarc[nrda + 1] = auxstore[deta];
                                                    nextpoolentry[nrda + 1] = nrdb;
                                                    nrdb = nrda;
                                                    deta--;
                                                }
                                                // obtaing the entry nrdsize from the pool
                                                nrdsize = queuenextpath[1];
                                                queuenextpath[1] = queuenextpath[nrdsize + 1];
                                                kpathlength[nrdsize + 1] = lenrst;
                                                queuefirstpath[nrdsize + 1] = nrdb;
                                                parm = nrdsize;
                                                invoke = true;
                                                goto iterate;
                                            }
                                        }
                                    }
                                }
                                force = false;
                                weight[edge3] -= large;
                                lengb += weight[edge3];
                                if (jterm != jnd2)
                                {
                                    jterm = (jedge > 0) ? nodei[jedge] : nodej[-jedge];
                                    jedge = shortpathtree[jterm];
                                }
                            } while (jterm != jnd2);
                            incrs1 = incrs2;
                        } while (nodep3 <= n);
                        // restore the join arcs
                        while (nodep2 <= nodep1 - 1)
                        {
                            j = auxstore[nodep2];
                            weight[j] -= large;
                            nodep2++;
                        }
                        // repeat with the next path
                        break;
                    }
                }
                # endregion
            }

            # endregion

            # region Format the output

            if (!getpaths)
            {
                // sort the paths
                hdb = poola;
                count = 0;
                do
                {
                    hda = hdb;
                    count++;
                    hdb = queuenextpath[hda + 1];
                    queuenextpath[hda + 1] = count;
                } while (hda != poolb);
                // release all queue entries to the pool
                j = queuesearch[nqp2 + 1];
                queuenextpath[j + 1] = queuenextpath[1];
                queuenextpath[1] = queuesearch[nqp1 + 1];
                nqp1 = 0;
                nqp2 = 0;
                hdb = 0;
                do
                {
                    j = hdb + 1;
                    hdb = queuenextpath[j];
                    queuenextpath[j] = 0;
                } while (hdb != 0);
                // exchanging all records
                jj = k + 2;
                for (i = 1; i <= jj; i++)
                {
                    while ((queuenextpath[i + 1] > 0) && (queuenextpath[i + 1] != i))
                    {
                        queorder = kpathlength[i + 1];
                        quefirst = queuefirstpath[i + 1];
                        quenext = queuenextpath[i + 1];
                        j = queuenextpath[i + 1] + 1;
                        kpathlength[i + 1] = kpathlength[j];
                        queuefirstpath[i + 1] = queuefirstpath[j];
                        queuenextpath[i + 1] = queuenextpath[j];
                        kpathlength[quenext + 1] = queorder;
                        queuefirstpath[quenext + 1] = quefirst;
                        queuenextpath[quenext + 1] = quenext;
                    }
                }
                kpathlength[1] = source;
                queuefirstpath[1] = sink;
                queuenextpath[1] = numpaths;
            }

            # endregion

            # region Construct the edges of the k shortest paths

            for (kk = 1; kk <= numpaths; kk++)
            {
                number = 0;
                if ((kk <= 0) || (kk > queuenextpath[1]))
                {
                    path[kk, 0] = number;
                    path[0, 0] = numpaths;
                    return;
                }
                index2 = kpathlength[1];
                length = kpathlength[kk + 1];
                sub3 = queuefirstpath[kk + 1];
                while (sub3 != 0)
                {
                    jsub = Math.Abs(sub3) + 1;
                    index3 = index2;
                    if (crossarc[jsub] > 0)
                    {
                        index1 = nodei[crossarc[jsub]];
                        index2 = nodej[crossarc[jsub]];
                    }
                    else
                    {
                        index1 = nodej[-crossarc[jsub]];
                        index2 = nodei[-crossarc[jsub]];
                    }
                    if (index2 != index3)
                    {
                        // store the arcs
                        sub2 = n;
                        arcnode[sub2] = crossarc[jsub];
                        while (index1 != index3)
                        {
                            sub1 = shortpathtree[index1];
                            sub2--;
                            if (sub2 > 0)
                                arcnode[sub2] = sub1;
                            else
                                nump = sub1;
                            index1 = (sub1 > 0) ? nodei[sub1] : nodej[-sub1];
                        }
                        while (sub2 <= n)
                        {
                            number++;
                            arcnode[number] = arcnode[sub2];
                            sub2++;
                        }
                    }
                    sub3 = nextpoolentry[jsub];
                }
                // 'number' is the number of edges in the path
                // 'lenght' is the lenght of the path
                // 'arcnode' is the array of edge numbers of the shortest path
                nextnode = source;
                count = 0;
                for (j = 1; j <= number; j++)
                {
                    i = arcnode[j];
                    count++;
                    if (nodei[i] == nextnode)
                    {
                        path[kk, count] = nextnode;
                        nextnode = nodej[i];
                    }
                    else
                    {
                        path[kk, count] = nodej[i];
                        nextnode = nodei[i];
                    }
                }
                count++;
                path[kk, count] = nextnode;
                path[kk, n + 1] = length;
                path[kk, 0] = count;
            }

            # endregion

            path[0, 0] = numpaths;
        }

        #endregion
    }
}
