using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;

namespace Penguin.PathFinding
{
    /// <summary>
    /// A class for finding a path through a 2D array of bools by stepping in true squares
    /// </summary>
    public class PathFinder
    {
        public bool GenerateCWorld { get; set; }
        private static int currentPath;
        public char[,] CWorld { get; set; }
        private static DateTime StartTime { get; set; } = DateTime.Now;
        private bool[,] Map { get; set; }
        private PathFindingNode[,] World { get; set; }

        /// <summary>
        /// Constructs a new instance using a defined map
        /// </summary>
        /// <param name="map">A 2D map of bools where true represents a valid step</param>
        public PathFinder(bool[,] map)
        {
            Map = map;

            if (GenerateCWorld)
            {
                ResetWorld();
                DumpCWorld();
            }
        }

        /// <summary>
        /// Constructs a new instance using a list of valid locations to step
        /// </summary>
        /// <param name="validNodes">A list of nodes representing valid locations to step</param>
        public PathFinder(List<Node> validNodes)
        {
            if (validNodes is null)
            {
                throw new ArgumentNullException(nameof(validNodes));
            }

            Map = new bool[(int)validNodes.Max(n => n.X), (int)validNodes.Max(n => n.Y)];

            foreach (Node n in validNodes)
            {
                Map[(int)n.X, (int)n.Y] = true;
            }
        }

        public void DumpCWorld()
        {
            StringBuilder sb = new();
            for (int y = 0; y < CWorld.GetLength(0); y++)
            {
                for (int x = 0; x < CWorld.GetLength(0); x++)
                {
                    _ = sb.Append(CWorld[x, y]);
                }
                _ = sb.Append(System.Environment.NewLine);
            }

            string LogPath = System.IO.Path.Combine(System.IO.Directory.GetCurrentDirectory(), $"{StartTime:yyyyMMdd_HHmmss}_{++currentPath}.log");
            Debug.WriteLine(LogPath);
            System.IO.File.WriteAllText(LogPath, sb.ToString());
        }

        /// <summary>
        /// Gets a path of nodes representing valid steps through the map generated at construction
        /// </summary>
        /// <param name="start">The start location</param>
        /// <param name="end">The intended end location</param>
        /// <returns>A list of valid steps, or null if no path is found</returns>
        public Node[] GetPath(Node start, Node end)
        {
            if (start is null)
            {
                throw new ArgumentNullException(nameof(start));
            }

            if (end is null)
            {
                throw new ArgumentNullException(nameof(end));
            }

            ResetWorld();

            foreach (PathFindingNode p in World)
            {
                p.Distance = Distance(p, end);
            }

            _ = RecursiveCheck(GetNodeByNode(start), GetNodeByNode(end));

            Node[] toReturn = CleanUp(start);

            if (GenerateCWorld)
            {
                if (toReturn != null)
                {
                    foreach (Node n in toReturn)
                    {
                        CWorld[(int)n.X, (int)n.Y] = '#';
                    }
                    DumpCWorld();
                }
            }

            return toReturn;
        }

        private static double Distance(Node a, Node b)
        {
            return Math.Sqrt(Math.Pow(Math.Abs(a.X - b.X), 2) + Math.Pow(Math.Abs(a.Y - b.Y), 2));
        }

        private PathFindingNode[] CleanUp(Node pc)
        {
            PathFindingNode startN = World[(int)pc.X, (int)pc.Y];

            if (startN.Steps == 0)
            {
                return null;
            }

            PathFindingNode toCheck = startN;

            List<PathFindingNode> toReturn = new()
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
                        List<PathFindingNode> newReturn = new();

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

        private PathFindingNode ClosestNeighborTo(PathFindingNode pc, PathFindingNode target)
        {
            List<string> vals = Neighbors(pc).OrderBy(p => Distance(p, target)).Select(p => $"{p.X}, {p.Y}, ({Distance(p, target)}").ToList();
            return Neighbors(pc).OrderBy(p => Distance(p, target)).FirstOrDefault(p => p.Viable && Distance(p, target) < Distance(pc, target));
        }

        private PathFindingNode GetNodeByNode(Node pc)
        {
            return World[(int)pc.X, (int)pc.Y];
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

        private PathFindingNode[] RecursiveCheck(PathFindingNode pc, PathFindingNode pe, int Length = 0)
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

        private void ResetWorld()
        {
            World = new PathFindingNode[Map.GetLength(0), Map.GetLength(1)];

            if (GenerateCWorld)
            {
                CWorld = new char[Map.GetLength(0), Map.GetLength(1)];
            }

            for (int x = 0; x < World.GetLength(0); x++)
            {
                for (int y = 0; y < World.GetLength(1); y++)
                {
                    bool n = Map[x, y];
                    PathFindingNode p = new(x, y)
                    {
                        Checked = false,
                        X = x,
                        Y = y,
                        Viable = n
                    };

                    if (GenerateCWorld)
                    {
                        CWorld[x, y] = n ? 'O' : 'X';
                    }

                    World[x, y] = p;
                }
            }
        }

        private PathFindingNode[] StraightLine(PathFindingNode pc, PathFindingNode target)
        {
            List<PathFindingNode> toreturn = new((int)(Math.Abs(pc.X - target.X) + Math.Abs(pc.Y - target.Y))) { pc };

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
    }
}