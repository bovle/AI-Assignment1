package problem;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class BFS {
    Queue<GridNode> queue = new LinkedList<>();
    List<GridNode> visited = new LinkedList<>();

    public List<GridNode> Solve(MovingBoxPlanner planner, Point2D start, double gridWidth, int listIndex, int boxIndex,
            List<Point2D> boxPath) {
        queue.clear();
        visited.clear();
        queue.add(new GridNode(planner, 0, start, gridWidth, listIndex, boxIndex));

        while (!queue.isEmpty()) {
            GridNode currentState = queue.poll();
            visited.add(currentState);
            if (isFree(currentState.pos, 0, boxPath, 0)) {
                LinkedList<GridNode> path = new LinkedList<>();
                path.add(currentState);
                while (currentState.parent != null) {
                    currentState = currentState.parent;
                    path.addFirst(currentState);
                }
                return path;
            } else {
                for (Neighbour n : currentState.neighbours) {
                    if (!n.isInList(visited)) {
                        GridNode newNeighbour = new GridNode(planner, currentState, n.gridInfo, 0, n.pos, gridWidth,
                                listIndex, boxIndex);
                        queue.add(newNeighbour);
                    }
                }
            }
        }
        System.err.println("no path for obstacle found");
        return null;
    }

    public boolean isFree(Point2D obstaclePos, double obstacleWidth, List<Point2D> boxPath, double boxWidth) {
        Rectangle2D obstacle = new Rectangle2D.Double(obstaclePos.getX() - (obstacleWidth / 2),
                obstaclePos.getY() - (obstacleWidth / 2), obstacleWidth, obstacleWidth);
        for (Point2D boxPos : boxPath) {
            Rectangle2D box = new Rectangle2D.Double(boxPos.getX() - (boxWidth / 2), boxPos.getY() - (boxWidth / 2),
                    boxWidth, boxWidth);
            if (obstacle.intersects(box))
                return false;
        }
        return true;
    }

}
