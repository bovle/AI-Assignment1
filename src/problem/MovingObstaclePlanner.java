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
  // because the obstalces have different width and hence all the other obstacles will have different sizes
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
  }

  public List<List<Point2D>> planAllPaths() {
    List<BoxPath> allPaths = new ArrayList<>();

    for (int i = 0; i < numMovingObs; i++) {
      int listIndex = listIndexes[i];
      BoxPath path = findBoxPath(listIndex, i);

      if (path == null) {
        return null;
      }
      allPaths.add(path);
    }
    return getResultPaths(allPaths);

  }

  private List<List<Point2D>> getResultPaths(List<BoxPath> allPaths) {
    List<List<Point2D>> result = new ArrayList<>();
    for (int i = 0; i < numMovingObs; i++) { // ordering.length
      BoxPath path = allPaths.get(i); //ordering[i]
      List<Point2D> pointPath = new ArrayList<>();
      pointPath.addAll(path.startPath);
      for (GridNode g : path.gridPath) {
        pointPath.add(g.pos);
      }
      pointPath.addAll(path.endPath);
      result.add(pointPath);
    }
    return result;
  }



  private BoxPath findBoxPath(int listIndex, int boxIndex) {
    System.out.println("*** Finding path for moving obstacle: " + boxIndex + " ***");

    double ow = obstacleWidths[boxIndex];

    boolean pathFound = false;
    double scalingFactor = Math.pow(2, listIndex - 1);
    BoxPath path = null;

    while (!pathFound && listIndex < 5) {

      if (listIndex > obstacles.get(boxIndex).numExtensions) {
        obstacles.get(boxIndex).extendObstacles(ow, scalingFactor);
      }
      path = findBoxPathAux(ow, scalingFactor, listIndex, boxIndex);
      if (path != null) {
        pathFound = true;
        double offset = (scalingFactor - 1) * ow / (scalingFactor * 2);
        Util.normalize(path, offset);
        //atGoal[boxIndex] = 1;
        // TODO: update obstacle to be at goal posiiton instead?
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



  public BoxPath findBoxPathAux(double ow, double scalingFactor, int listIndex, int boxIndex) {
    double gw = ow / scalingFactor;
    double offset = (scalingFactor - 1) * gw / 2;
    List<Point2D> startPath = new ArrayList<>();
    List<Point2D> endPath = new ArrayList<>();

    double originalStartX = problemSpec.getMovingObstacles().get(boxIndex).getRect().getCenterX();
    double originalStartY = problemSpec.getMovingObstacles().get(boxIndex).getRect().getCenterY();

    Point2D shrinkedStart = new Point2D.Double(originalStartX - offset, originalStartY - offset);


    startPath = pointToGridCenter(shrinkedStart, gw, listIndex, boxIndex);
    if (startPath == null) {
      return null;
    }

    Point2D centeredStart = startPath.get(startPath.size() - 1); // get last element in list

    BFS agent = new BFS();
    /* TODO: figure out how BFS works */
    // List<Point2D> boxPath
    List<GridNode> path = (List<GridNode>) agent.Solve(this, centeredStart, ow, listIndex, boxIndex, boxPath);
    if (path == null)
      return null;

    return new BoxPath(path, startPath, null); //no end path
  }

  public GridInfo isObstacle(Point2D p, int listIndex, int boxIndex) {
    return this.obstacles.get(boxIndex).isObstacle(p, listIndex, boxIndex);
  }

  private List<Point2D> pointToGridCenter(Point2D point, double gw, int listIndex, int boxIndex) {
    double x = point.getX();
    double y = point.getY();
    List<Point2D> corners = new ArrayList<>();
    corners.add(new Point2D.Double(x - gw / 2, y + gw / 2)); // topLeft
    corners.add(new Point2D.Double(x + gw / 2, y + gw / 2)); // topRight
    corners.add(new Point2D.Double(x - gw / 2, y - gw / 2)); // bottomLeft
    corners.add(new Point2D.Double(x + gw / 2, y - gw / 2)); // bottomRight

    List<Point2D> gridCenters = new ArrayList<>();
    Point2D currentCenter;
    int notFreeCount = 0;
    for (Point2D p : corners) {
      currentCenter = Util.getGridCenter(p, gw);
      if (GridType.FREE == isObstacle(currentCenter, listIndex, boxIndex).type) { // need to handle start/goal and
                                                                                  // movable obstacles here
        gridCenters.add(currentCenter);
      } else {
        gridCenters.add(null);
        notFreeCount++;
      }
    }
    if (notFreeCount == 4) {
      return null;
    }

    // TODO: better way to choose a "random" gridcenter
    Point2D startInGrid;
    for (Point2D p : gridCenters) {
      if (p != null) {
        startInGrid = p;
        break;
      }
    }

    // sometimes the middlePoint is the same as one of the others
    Point2D middlePoint = new Point2D.Double(x, startInGrid.getY());
    List<Point2D> path = new ArrayList<>();
    path.add(point);
    if (!Util.equalPositions(point, middlePoint)) {
      path.add(middlePoint);
    }
    path.add(startInGrid);
    return path;
  }

/*
  private List<Integer> obstaclesInPath(List<Point2D> path, Integer listIndex, Integer boxIndex) {
     List<Integer> obstacleIndex = new ArrayList<>();
     for (Point2D p : path) {
       if (GridType.MOV_OBS == boxPlanner.isObstacle(p, listIndex, boxIndex)) {
         obstacleIndex.add(getMovObsIndex(p, listIndex));
       }
     }
     return obstacleIndex;
 }

   private Integer getMovObsIndex(Point2D p, Integer listIndex) {
     for (int i = 0; i < numMovingObstacles; i++) {
       Rectangle2D currentObstacle = boxPlanner.movingObstacles.get(listIndex).get(i);
       if (currentObstacle.contains(p)) {
         return i;
       }
     }
     return -1; // something wrong
   }
   */


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
