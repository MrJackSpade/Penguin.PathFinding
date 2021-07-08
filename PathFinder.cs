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
        public bool GenerateCWorld { get; set; } = false;
        private static int currentPath = 0;
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
            this.Map = map;

            if (this.GenerateCWorld)
            {
                this.ResetWorld();
                this.DumpCWorld();
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

            this.Map = new bool[(int)validNodes.Max(n => n.X), (int)validNodes.Max(n => n.Y)];

            foreach (Node n in validNodes)
            {
                this.Map[(int)n.X, (int)n.Y] = true;
            }
        }

        public void DumpCWorld()
        {
            StringBuilder sb = new StringBuilder();
            for (int y = 0; y < this.CWorld.GetLength(0); y++)
            {
                for (int x = 0; x < this.CWorld.GetLength(0); x++)
                {
                    sb.Append(this.CWorld[x, y]);
                }
                sb.Append(System.Environment.NewLine);
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

            this.ResetWorld();

            foreach (PathFindingNode p in this.World)
            {
                p.Distance = Distance(p, end);
            }

            this.RecursiveCheck(this.GetNodeByNode(start), this.GetNodeByNode(end));

            Node[] toReturn = this.CleanUp(start);

            if (this.GenerateCWorld)
            {
                if (toReturn != null)
                {
                    foreach (Node n in toReturn)
                    {
                        this.CWorld[(int)n.X, (int)n.Y] = '#';
                    }
                    this.DumpCWorld();
                }
            }

            return toReturn;
        }

        private static double Distance(Node a, Node b) => Math.Sqrt(Math.Pow(Math.Abs(a.X - b.X), 2) + Math.Pow(Math.Abs(a.Y - b.Y), 2));

        private PathFindingNode[] CleanUp(Node pc)
        {
            PathFindingNode startN = this.World[(int)pc.X, (int)pc.Y];

            if (startN.Steps == 0)
            {
                return null;
            }

            PathFindingNode toCheck = startN;

            List<PathFindingNode> toReturn = new List<PathFindingNode>()
            {
                startN
            };

            while (toCheck.Steps != 0)
            {
                toCheck = this.Neighbors(toCheck).Where(p => p.Checked).OrderBy(p => p.Steps).First();

                toReturn.Add(toCheck);
            }

            int end = toReturn.Count - 1;

            for (int start = 0; start < end; start++)
            {
                for (; end > 0; end--)
                {
                    PathFindingNode[] skip = this.StraightLine(toReturn[start], toReturn[end]);

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

        private PathFindingNode ClosestNeighborTo(PathFindingNode pc, PathFindingNode target)
        {
            List<string> vals = this.Neighbors(pc).OrderBy(p => Distance(p, target)).Select(p => $"{p.X}, {p.Y}, ({Distance(p, target)}").ToList();
            return this.Neighbors(pc).OrderBy(p => Distance(p, target)).FirstOrDefault(p => p.Viable && Distance(p, target) < Distance(pc, target));
        }

        private PathFindingNode GetNodeByNode(Node pc) => this.World[(int)pc.X, (int)pc.Y];

        private IEnumerable<PathFindingNode> Neighbors(Node pc)
        {
            for (double cx = pc.X - 1; cx <= pc.X + 1; cx++)
            {
                for (double cy = pc.Y - 1; cy <= pc.Y + 1; cy++)
                {
                    if (cx < 0 || cx > this.World.GetLength(0) - 1 || cy < 0 || cy > this.World.GetLength(1) - 1)
                    {
                        continue;
                    }

                    if (cx == pc.X && cy == pc.Y)
                    {
                        continue;
                    }

                    yield return this.World[(int)cx, (int)cy];
                }
            }
        }

        private PathFindingNode[] RecursiveCheck(PathFindingNode pc, PathFindingNode pe, int Length = 0)
        {
            IEnumerable<PathFindingNode> options = this.Neighbors(pc).Where(p => p.Viable && !p.Checked);

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

                PathFindingNode[] result = this.RecursiveCheck(px, pe, Length + 1);

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
            this.World = new PathFindingNode[this.Map.GetLength(0), this.Map.GetLength(1)];

            if (this.GenerateCWorld)
            {
                this.CWorld = new char[this.Map.GetLength(0), this.Map.GetLength(1)];
            }

            for (int x = 0; x < this.World.GetLength(0); x++)
            {
                for (int y = 0; y < this.World.GetLength(1); y++)
                {
                    bool n = this.Map[x, y];
                    PathFindingNode p = new PathFindingNode(x, y)
                    {
                        Checked = false,
                        X = x,
                        Y = y,
                        Viable = n
                    };

                    if (this.GenerateCWorld)
                    {
                        this.CWorld[x, y] = n ? 'O' : 'X';
                    }

                    this.World[x, y] = p;
                }
            }
        }

        private PathFindingNode[] StraightLine(PathFindingNode pc, PathFindingNode target)
        {
            List<PathFindingNode> toreturn = new List<PathFindingNode>((int)(Math.Abs(pc.X - target.X) + Math.Abs(pc.Y - target.Y))) { pc };

            PathFindingNode next = this.ClosestNeighborTo(pc, target);

            while (next != null)
            {
                if (next == target)
                {
                    return toreturn.ToArray();
                }

                toreturn.Add(next);

                next = this.ClosestNeighborTo(next, target);
            }

            return null;
        }
    }
}