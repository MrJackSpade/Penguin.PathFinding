namespace Penguin.PathFinding
{
    internal class PathFindingNode : Node
    {
        public bool Checked { get; set; }

        public double Distance { get; set; }

        public int Steps { get; set; }

        public bool Viable { get; set; }

        public PathFindingNode(double x, double y) : base(x, y)
        {
        }

        public PathFindingNode(int x, int y) : base(x, y)
        {
        }

        public PathFindingNode(Node node) : base(node.X, node.Y)
        {
        }
    }
}