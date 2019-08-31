using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Penguin.PathFinding
{
    /// <summary>
    /// A class for finding a path through a 2D array of bools by stepping in true squares
    /// </summary>
    public class PathFinder
    {

        private PathFindingNode[,] World { get; set; }

        private bool[,] Map { get; set; }

        /// <summary>
        /// Constructs a new instance using a defined map
        /// </summary>
        /// <param name="map">A 2D map of bools where true represents a valid step</param>
        public PathFinder(bool[,] map)
        {
            Map = map;
        }

        /// <summary>
        /// Constructs a new instance using a list of valid locations to step
        /// </summary>
        /// <param name="validNodes">A list of nodes representing valid locations to step</param>
        public PathFinder(List<Node> validNodes)
        {
            Map = new bool[(int)validNodes.Max(n => n.X), (int)validNodes.Max(n => n.Y)];

            foreach(Node n in validNodes)
            {
                Map[(int)n.X, (int)n.Y] = true;
            }
        }

        private void ResetWorld()
        {
            World = new PathFindingNode[Map.GetLength(0), Map.GetLength(1)];

            for (int x = 0; x < World.GetLength(0); x++)
            {
                for (int y = 0; y < World.GetLength(1); y++)
                {
                    bool n = Map[x, y];
                    PathFindingNode p = new PathFindingNode(x, y)
                    {
                        Checked = false,
                        X = x,
                        Y = y,
                        Viable = n
                    };

                    World[x, y] = p;
                }
            }
        }

        /// <summary>
        /// Gets a path of nodes representing valid steps through the map generated at construction
        /// </summary>
        /// <param name="start">The start location</param>
        /// <param name="end">The intended end location</param>
        /// <returns>A list of valid steps, or null if no path is found</returns>
        public Node[] GetPath(Node start, Node end)
        {
            ResetWorld();

            foreach (PathFindingNode p in World)
            {
                p.Distance = Distance(p, end);
            }

            RecursiveCheck(GetNodeByNode(start), GetNodeByNode(end));

            return CleanUp(start);
        }

        private PathFindingNode GetNodeByNode(Node pc)
        {
            return World[(int)pc.X, (int)pc.Y];
        }

        PathFindingNode ClosestNeighborTo(PathFindingNode pc, PathFindingNode target)
        {
            List<string> vals = Neighbors(pc).OrderBy(p => Distance(p, target)).Select(p => $"{p.X}, {p.Y}, ({Distance(p, target)}").ToList();
            return Neighbors(pc).OrderBy(p => Distance(p, target)).FirstOrDefault(p => p.Viable && Distance(p, target) < Distance(pc, target));
        }

        private IEnumerable<PathFindingNode> Neighbors(Node pc)
        {
            for (double cx = pc.X - 1; cx <= pc.X + 1; cx++)
            {
                for (double cy = pc.Y - 1; cy <= pc.Y + 1; cy++)
                {
                    if (cx < 0 || cx > World.GetLength(0) - 1 || cy < 0 || cy > World.GetLength(1) - 1)
                    {
                        continue;
                    }

                    if (cx == pc.X && cy == pc.Y)
                    {
                        continue;
                    }

                    yield return World[(int)cx, (int)cy];
                }
            }
        }
        PathFindingNode[] RecursiveCheck(PathFindingNode pc, PathFindingNode pe, int Length = 0)
        {
            IEnumerable<PathFindingNode> options = Neighbors(pc).Where(p => p.Viable && !p.Checked);

            PathFindingNode endNode = options.FirstOrDefault(pn => pn == pe);

            if (endNode != null)
            {
                endNode.Checked = true;
                pc.Steps = 1;
                PathFindingNode[] result = new PathFindingNode[Length + 2];
                result[Length + 1] = endNode;
                result[Length] = pc;
                return result;
            }

            foreach (PathFindingNode px in options.OrderBy(py => py.Distance))
            {
                px.Checked = true;

                PathFindingNode[] result = RecursiveCheck(px, pe, Length + 1);

                if (result != null)
                {
                    result[Length] = pc;
                    pc.Steps = result[Length + 1].Steps + 1;

                    return result;
                }
            }

            return null;
        }


        private static double Distance(Node a, Node b)
        {
            return Math.Sqrt(Math.Pow(Math.Abs(a.X - b.X), 2) + Math.Pow(Math.Abs(a.Y - b.Y), 2));
        }
        private PathFindingNode[] StraightLine(PathFindingNode pc, PathFindingNode target)
        {

            List<PathFindingNode> toreturn = new List<PathFindingNode>((int)(Math.Abs(pc.X - target.X) + Math.Abs(pc.Y - target.Y))) { pc };

            PathFindingNode next = ClosestNeighborTo(pc, target);

            while (next != null)
            {
                if (next == target)
                {
                    return toreturn.ToArray();
                }

                toreturn.Add(next);

                next = ClosestNeighborTo(next, target);
            }

            return null;
        }
        private PathFindingNode[] CleanUp(Node pc)
        {
            PathFindingNode startN = World[(int)pc.X, (int)pc.Y];

            PathFindingNode toCheck = startN;

            List<PathFindingNode> toReturn = new List<PathFindingNode>()
            {
                startN
            };

            while (toCheck.Steps != 0)
            {
                toCheck = Neighbors(toCheck).Where(p => p.Checked).OrderBy(p => p.Steps).First();

                toReturn.Add(toCheck);
            }

            int end = toReturn.Count - 1;

            for (int start = 0; start < end; start++)
            {
                for (; end > 0; end--)
                {
                    PathFindingNode[] skip = StraightLine(toReturn[start], toReturn[end]);

                    if (skip != null && skip.Length < end - start)
                    {
                        List<PathFindingNode> newReturn = new List<PathFindingNode>();

                        newReturn.AddRange(toReturn.Take(start));

                        newReturn.AddRange(skip);

                        start = newReturn.Count;

                        newReturn.AddRange(toReturn.Skip(end));

                        toReturn = newReturn;

                        end = toReturn.Count - 1;
                    }
                }

            }

            return toReturn.ToArray();
        }
    }
}
