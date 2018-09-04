package problem;

import java.awt.geom.Point2D;


public class GridNode implements Comparable<GridNode> {

  public GridNode parent;
  public GridType type;
  public double gCost; // cost from start to current node
  public double hCost; // cost from current node to goal
  public double fCost; // gCost + hCost
  public int depth;
  public Point2D pos; // position of this GridNode, CENTER
  public Neighbour neighbours[]; //todo

  /**
   * Creates a root gridNode, the starting node
   */
  public GridNode(double hCost, Point2D pos, double gridWidth) {
    this.parent = null;
    this.type = GridType.FREE; /* assuming we don't start on a obstacle */
    this.gCost = 0;
    this.hCost = hCost;
    this.fCost = this.gCost + this.hCost;
    this.depth = 0;
    this.pos = pos;
    this.neighbours = new Neighbour[4];
    this.setNeighbours(gridWidth);
  }

  /**
   * Creates a non-root gridNode,
   */
  public GridNode(GridNode parent, GridType type, double hCost, Point2D pos, double gridWidth) {
    this.parent = parent;
    /* gCost: assume the cost increases with the width of the "gridcell" for every step you take.
     * todo: Room for improvement: extra cost if making a turn or if it's a moving obstacle
     */
    this.type = type;
    this.gCost = parent.gCost + gridWidth;
    this.hCost = hCost;
    this.fCost = this.gCost + this.hCost;
    this.depth = 0;
    this.pos = pos;
    this.neighbours = new Neighbour[4];
    this.setNeighbours(gridWidth);
  }


  private void setNeighbours(double gridWidth) {
    double currentX = this.pos.getX();
    double currentY = this.pos.getY();
    Point2D[] points = {
      new Point2D.Double(currentX,             currentY + gridWidth),
      new Point2D.Double(currentX + gridWidth, currentY),
      new Point2D.Double(currentX,             currentY - gridWidth),
      new Point2D.Double(currentX - gridWidth, currentY),
    };

    for (int i = 0; i < 4; i++) {
      this.neighbours[i] = new Neighbour(points[i], Util.findGridType(points[i]));
    }
  }

  /**
   * Compares the F-cost of this node to the F-cost of node g.
   *
   * Defining a "compareTo" method is necessary in order for gridNodes
   * to be used in a priority queue.
   *
   * @param g node to compare path cost with
   * @return  -1 if this node has a lower total path cost than node s
   *          0 if this node has the same total path cost as node s
   *          1 if this node has a greater total path cost than node s
   */
  public int compareTo(GridNode g) {
      return Double.compare(this.fCost, g.fCost);
  }


}
