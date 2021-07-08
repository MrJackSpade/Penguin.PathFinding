namespace Penguin.PathFinding
{
    /// <summary>
    /// Represents an X/Y coordinate pair
    /// </summary>
    public class Node
    {
        /// <summary>
        /// The X location
        /// </summary>
        public double X;

        /// <summary>
        /// The Y location
        /// </summary>
        public double Y;

        /// <summary>
        /// Constructs a new instance from the given X/Y
        /// </summary>
        /// <param name="x">The X location</param>
        /// <param name="y">The Y location</param>
        public Node(double x, double y)
        {
            (this.X, this.Y) = (x, y);
        }

        /// <summary>
        /// Constructs a new instance from the given X/Y
        /// </summary>
        /// <param name="x">The X location</param>
        /// <param name="y">The Y location</param>
        public Node(int x, int y)
        {
            (this.X, this.Y) = (x, y);
        }
    }
}