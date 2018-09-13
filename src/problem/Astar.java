package problem;

import java.awt.geom.Point2D;
import java.lang.Math;
import java.util.Collections;
import java.util.LinkedList;
import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

public class Astar {
  private PriorityQueue<GridNode> open;
  private List<GridNode> evaluated;

  public Astar() {
    this.open = new PriorityQueue<GridNode>();
    this.evaluated = new ArrayList<GridNode>();
  }

  public List<GridNode> search(MovingBoxPlanner planner, Point2D start, Point2D goal, double gridWidth, int listIndex,
      int boxIndex) {
    // hCost from start to goal: manhattan distance
    double hCost = calculateHcost(start, goal);
    open.add(new GridNode(planner, hCost, start, gridWidth, listIndex, boxIndex));

    while (open.size() > 0) {
      GridNode currentNode = (GridNode) open.poll();

      // if goal found
      if (Util.equalPositions(currentNode.pos, goal)) {
        List<GridNode> pathToGoal = new LinkedList<GridNode>();
        while (currentNode.parent != null) {
          pathToGoal.add(currentNode);
          currentNode = currentNode.parent;
        }
        // pathToGoal.add(currentNode);
        Collections.reverse(pathToGoal);

        // reset for next search
        reset();
        // todo: the start position is not in the path for some reason
        return pathToGoal;
      }

      // For each neighbour...
      for (Neighbour n : currentNode.neighbours) {
        if ((n.gridInfo.type != GridType.STAT_OBS) && !n.isInList(this.evaluated)) {
          GridNode neighbourInOpen = n.isInPrioQueue(this.open);
          // todo: improve the calculation of cost
          double newPathCost = currentNode.gCost + gridWidth;
          if (n.gridInfo.type != GridType.FREE) {
            newPathCost += 1000; // only go through movable obstacles or start/goal positions if needed
          }

          if (neighbourInOpen == null || (neighbourInOpen != null && newPathCost < neighbourInOpen.gCost)) {
            if (neighbourInOpen != null) {
              neighbourInOpen.gCost = newPathCost;
              neighbourInOpen.fCost = neighbourInOpen.hCost + neighbourInOpen.gCost;
              neighbourInOpen.parent = currentNode;
            } else {
              double newHcost = calculateHcost(n.pos, goal);
              // todo: is a new neighbour always FREE?
              GridNode newNeighbour = new GridNode(planner, currentNode, n.gridInfo, newHcost, n.pos, gridWidth,
                  listIndex, boxIndex);
              this.open.add(newNeighbour);
            }
          }
        }
      }
      this.evaluated.add(currentNode);
    }
    // no path found
    System.out.println("No path found");
    return null;
  }

  private double calculateHcost(Point2D start, Point2D goal) {
    return Util.manhattanDist(start, goal);
  }

  /**
   * Resets the search agent (clears instance variables to be ready for next
   * search request).
   */
  private void reset() {
    open.clear();
    this.evaluated = new ArrayList<GridNode>();
    // todo: is there a better wya to clear an ArrayList?
  }

}
