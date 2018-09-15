package problem;

import static tester.Tester.*; // to use MAX_ERROR

import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.lang.*;

public class MovingObstaclePlanner implements PathPlanner {
  private ProblemSpec problemSpec;
  private MovingBoxPlanner boxPlanner;
  // every list represent the obstacles for a specific moving obstacle.
  // because the obstalces have different width and hence all the other obstacles
  // will have different sizes
  private List<Obstacles> obstacles;
  private List<List<Rectangle2D>> boxPaths;
  private Double[] obstacleWidths;
  private Integer numMovingObs;
  private Integer[] listIndexes;

  public MovingObstaclePlanner(ProblemSpec ps, MovingBoxPlanner boxPlanner, List<List<Point2D>> boxPaths) {
    this.problemSpec = ps;
    this.boxPlanner = boxPlanner;
    this.boxPaths = pointPathsToRectanglePaths(boxPaths);

    // initilizing atGoal to 0
    this.numMovingObs = problemSpec.getMovingObstacles().size();
    this.obstacleWidths = new Double[numMovingObs];
    for (int i = 0; i < numMovingObs; i++) {
      obstacleWidths[i] = ps.getMovingObstacles().get(i).getRect().getWidth();
    }
    this.listIndexes = new Integer[numMovingObs];
    for (int i = 0; i < numMovingObs; i++) {
      listIndexes[i] = 1;
    }

    this.obstacles = new ArrayList<>();
    for (int i = 0; i < numMovingObs; i++) {
      this.obstacles.add(new Obstacles(boxPlanner, i, this.boxPaths));
    }
  }

  public List<List<Point2D>> planAllPaths() {
    List<List<GridNode>> allPaths = new ArrayList<>();

    for (int boxIndex = 0; boxIndex < numMovingObs; boxIndex++) {
      int listIndex = listIndexes[boxIndex];
      List<GridNode> path = findBoxPath(listIndex, boxIndex);

      if (path == null) {
        return null;
      }
      allPaths.add(path);
    }
    return getResultPaths(allPaths);

  }

  private List<List<Point2D>> getResultPaths(List<List<GridNode>> allPaths) {
    List<List<Point2D>> result = new ArrayList<>();
    for (int i = 0; i < numMovingObs; i++) { // ordering.length
      List<GridNode> path = allPaths.get(i); // ordering[i]
      List<Point2D> pointPath = new ArrayList<>();
      for (GridNode g : path) {
        pointPath.add(g.pos);
      }
      result.add(pointPath);
    }
    return result;
  }

  private List<GridNode> findBoxPath(int listIndex, int boxIndex) {
    System.out.println("*** Finding path for moving obstacle: " + boxIndex + " ***");
    double ow = obstacleWidths[boxIndex];
    boolean pathFound = false;
    double scalingFactor = Math.pow(2, listIndex - 1);
    List<GridNode> path = null;

    while (!pathFound && listIndex < 5) {

      if (listIndex > obstacles.get(boxIndex).numExtensions) {
        obstacles.get(boxIndex).extendObstacles(ow, scalingFactor);
      }
      path = findBoxPathAux(ow, scalingFactor, listIndex, boxIndex);
      if (path != null) {
        pathFound = true;
        double offset = (scalingFactor - 1) * ow / (scalingFactor * 2);
        Util.normalize(path, offset);
        // atGoal[boxIndex] = 1;
        // update obstacle to be at goal position instead?
        Point2D goalPos = path.get(path.size() - 1).pos;
        for (int i = boxIndex; i < numMovingObs; i++) {
          this.obstacles.get(i).updateObstacle(goalPos);
        }
        listIndexes[boxIndex] = listIndex;
      } else {
        listIndex++;
        scalingFactor = Math.pow(2, listIndex - 1);
        System.out.println("New gridsize: " + ow / scalingFactor);
      }
    }
    if (!pathFound) {
      path = null;
    }
    return path;
  }

  public List<GridNode> findBoxPathAux(double ow, double scalingFactor, int listIndex, int boxIndex) {
    double gw = ow / scalingFactor;
    double offset = (scalingFactor - 1) * gw / 2;
    List<GridNode> startPath = new ArrayList<>();
    List<GridNode> endPath = new ArrayList<>();

    double originalStartX = problemSpec.getMovingObstacles().get(boxIndex).getRect().getCenterX();
    double originalStartY = problemSpec.getMovingObstacles().get(boxIndex).getRect().getCenterY();

    Point2D shrinkedStart = new Point2D.Double(originalStartX - offset, originalStartY - offset);
    shrinkedStart = Util.roundToGrid(shrinkedStart, 0.000001);
    GridNode shrinkedStartNode = new GridNode(this, 0, shrinkedStart, gw, listIndex, boxIndex);

    startPath = pointToGridCenter(shrinkedStartNode, gw, listIndex, boxIndex);
    if (startPath == null) {
      return null;
    }

    Point2D centeredStart = startPath.get(startPath.size() - 1).pos; // get last element in list

    BFS agent = new BFS();
    /* TODO: figure out how BFS works */
    // List<Point2D> boxPath
    List<GridNode> midPath = (List<GridNode>) agent.Solve(this, centeredStart, gw, listIndex, boxIndex);

    if (midPath == null)
      return null;

    midPath.remove(0);

    List<GridNode> path = new ArrayList<>();
    path.addAll(startPath);
    path.addAll(midPath);

    return path; // no end path
  }

  public GridInfo isObstacle(Point2D p, int listIndex, int boxIndex) {
    return this.obstacles.get(boxIndex).isObstacle(p, listIndex, boxIndex);
  }

  private List<GridNode> pointToGridCenter(GridNode startPoint, double gw, int listIndex, int boxIndex) {
    double x = startPoint.pos.getX();
    double y = startPoint.pos.getY();
    List<Point2D> corners = new ArrayList<>();
    corners.add(new Point2D.Double(x - gw / 2, y + gw / 2)); // topLeft
    corners.add(new Point2D.Double(x + gw / 2, y + gw / 2)); // topRight
    corners.add(new Point2D.Double(x - gw / 2, y - gw / 2)); // bottomLeft
    corners.add(new Point2D.Double(x + gw / 2, y - gw / 2)); // bottomRight
    corners.add(startPoint.pos); // center

    List<GridNode> gridCenters = new ArrayList<>();
    Point2D currentCenter;
    int statObstacleCount = 0;
    for (Point2D p : corners) {
      currentCenter = Util.getGridCenter(p, gw);
      GridInfo gridInfo = isObstacle(currentCenter, listIndex, boxIndex);
      if (gridInfo.type != GridType.STAT_OBS) {
        gridCenters.add(new GridNode(this, null, gridInfo, 0, currentCenter, gw, listIndex, boxIndex));
      } else {
        gridCenters.add(null);
        statObstacleCount++;
      }
    }
    if (statObstacleCount == 5) {
      System.out.println("no free space");
      return null;
    }

    // TODO: better way to choose a "random" gridcenter
    GridNode startInGrid = null;
    for (GridNode p : gridCenters) {
      if (p != null) {
        startInGrid = p;
        break;
      }
    }
    Point2D middlePoint = new Point2D.Double(x, startInGrid.pos.getY());
    List<GridNode> path = new ArrayList<>();
    path.add(startPoint);
    if (!Util.equalPositions(startPoint.pos, middlePoint)) {
      GridInfo middlePointInfo = isObstacle(middlePoint, listIndex, boxIndex);
      path.add(new GridNode(this, null, middlePointInfo, 0, middlePoint, gw, listIndex, boxIndex));
    }
    if (!Util.equalPositions(startInGrid.pos, middlePoint)) {
      path.add(startInGrid);
    }
    return path;
  }

  private List<List<Rectangle2D>> pointPathsToRectanglePaths(List<List<Point2D>> boxPaths) {
    List<List<Rectangle2D>> allPaths = new ArrayList<>();
    double ow = problemSpec.getRobotWidth();

    for (List<Point2D> path : boxPaths) {
      List<Rectangle2D> rectanglePath = new ArrayList<>();
      for (Point2D p : path) {
        Rectangle2D currentRect = Util.pointToRect(p, ow);
        rectanglePath.add(currentRect);
      }
      allPaths.add(rectanglePath);
    }
    return allPaths;
  }
}
