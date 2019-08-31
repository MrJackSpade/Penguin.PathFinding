using System;
using System.Collections.Generic;
using System.Text;

namespace Penguin.PathFinding
{
    class PathFindingNode : Node
    {
        public PathFindingNode(double x, double y) : base(x, y)
        {
        }

        public PathFindingNode(int x, int y) : base(x, y)
        {
        }

        public PathFindingNode(Node node) : base(node.X, node.Y)
        {
        }

        public bool Checked { get; set; }
        public bool Viable { get; set; }
        public int Steps { get; set; }
        public Double Distance { get; set; }
    }
}
