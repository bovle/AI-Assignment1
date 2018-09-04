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
  private Point2D start;
  private Point2D goal;

  public Astar(Point2D start, Point2D goal) {
    this.start = start;
    this.goal = goal;
    this.open = new PriorityQueue<GridNode>();
    this.evaluated = new ArrayList<GridNode>();
  }

  public List<GridNode> search() {
    // hCost from start to goal: manhattan distance
    double hCost = calculateHcost(this.start, this.goal);
    open.add(new GridNode(hCost, start));

    while(open.size() > 0) {
      GridNode currentNode = (GridNode)open.poll();

      // if goal found
      if (Util.equalPositions(currentNode.pos, goal)) {
        List<GridNode> pathToGoal = new LinkedList<GridNode>();
        while (currentNode.parent != null) {
            pathToGoal.add(currentNode);
            currentNode = currentNode.parent;
        }
        Collections.reverse(pathToGoal);

        // reset for next search
        reset();
        // todo: the start position is not in the path for some reason
        return pathToGoal;
      }

      // For each neighbour...
      for(Neighbour n : currentNode.neighbours) {
        if(!(n.type == GridType.STAT_OBS || n.isInList(this.evaluated))) {
          GridNode neighbourInOpen = n.isInPrioQueue(this.open);
          double newPathCost = currentNode.gCost + 0.05;

          if(neighbourInOpen == null || ( neighbourInOpen != null && newPathCost < neighbourInOpen.gCost)) {
            if(neighbourInOpen != null) {
              neighbourInOpen.gCost = newPathCost;
              neighbourInOpen.fCost = neighbourInOpen.hCost + neighbourInOpen.gCost;
              neighbourInOpen.parent = currentNode;
            }
            else {
              double newHcost = calculateHcost(n.pos, this.goal);
              GridNode newNeighbour = new GridNode(currentNode, GridType.FREE, newHcost, n.pos);
              this.open.add(newNeighbour);
            }
          }
        }
      }
    }
    // no path found
    System.out.println("No path found");
    return null;
  }


  private double calculateHcost(Point2D start, Point2D goal) {
    return Math.abs(start.getX() - goal.getX()) + Math.abs(start.getY() - goal.getY());
  }

  /**
   * Resets the search agent (clears instance variables to be ready for next
   * search request).
   */
  private void reset() {
      open.clear();
      // todo
  }

}
