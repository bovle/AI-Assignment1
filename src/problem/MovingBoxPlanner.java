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

public class MovingBoxPlanner implements PathPlanner {
  private ProblemSpec problemSpec;
  // private List<Point2D> currentStaticObstacles;
  private List<List<Rectangle2D>> staticObstacles;
  private List<List<Rectangle2D>> movingObstacles;
  private List<List<Rectangle2D>> boxesStart;
  private List<List<Rectangle2D>> boxesGoal;
  private Integer numMovingBoxes;
  private Integer[] atGoal;
  private Integer[] ordering; // the order the boxes are going to move
  private List<OrderObject> orderObjects;
  private Integer[] listIndexes; // the index where the algorithm finds a path
  private Integer numExtensions;
  private List<Rectangle2D> temporaryObstacles;

  public MovingBoxPlanner(ProblemSpec ps) {
    this.problemSpec = ps;
    this.ordering = new Integer[ps.getNumMovingBoxes()];
    this.orderObjects = new ArrayList<>();
    // static obstacles
    List<Rectangle2D> statObsOriginal = new ArrayList<>();
    for (StaticObstacle box : ps.getStaticObstacles()) {
      statObsOriginal.add(Util.grow(box.getRect(), MAX_ERROR));
    }
    this.staticObstacles = new ArrayList<>();
    this.staticObstacles.add(statObsOriginal);

    // moving obstacles
    List<Rectangle2D> movObsOriginal = new ArrayList<>();
    for (Box box : ps.getMovingObstacles()) {
      movObsOriginal.add(Util.grow(box.getRect(), MAX_ERROR));
    }
    this.movingObstacles = new ArrayList<>();
    this.movingObstacles.add(movObsOriginal);

    // number of moving boxesStart
    this.numMovingBoxes = ps.getNumMovingBoxes();

    // moving boxes starting Rectangles
    List<Rectangle2D> boxStartOriginal = new ArrayList<>();
    for (Box box : ps.getMovingBoxes()) {
      boxStartOriginal.add(Util.grow(box.getRect(), MAX_ERROR));
    }
    this.boxesStart = new ArrayList<>();
    this.boxesStart.add(boxStartOriginal);

    // moving boxes goal Rectangles
    List<Rectangle2D> boxEndOriginal = new ArrayList<>();
    Point2D endCenterPoint;
    double w = ps.getRobotWidth();
    for (Point2D endCenter : ps.getMovingBoxEndPositions()) {
      Rectangle2D endBox = new Rectangle2D.Double(endCenter.getX(), endCenter.getY(), w, w);
      Rectangle2D endBoxGrown = Util.grow(endBox, MAX_ERROR);
      boxEndOriginal.add(endBoxGrown);
    }
    this.boxesGoal = new ArrayList<>();
    this.boxesGoal.add(boxEndOriginal);

    // initilizing atGoal to 0
    this.atGoal = new Integer[numMovingBoxes];
    for (int i = 0; i < numMovingBoxes; i++) {
      atGoal[i] = 0;
    }

    // initializing all listIndexes to 1
    this.listIndexes = new Integer[numMovingBoxes];
    for (int i = 0; i < numMovingBoxes; i++) {
      listIndexes[i] = 1;
    }

    // temporaryObstacles
    this.temporaryObstacles = new ArrayList<>();

    this.numExtensions = 0;

  }

  public List<List<Rectangle2D>> getMovingObstacles() {
    return this.movingObstacles;
  }
  public List<List<Rectangle2D>> getStaticObstacles() {
    return this.staticObstacles;
  }
  public List<List<Rectangle2D>> getBoxesStart() {
    return this.boxesStart;
  }

  // p1 and p2 are the end points of the robot line
  // public List<Point2D> findNewPath(Point2D p1, Point2D p2, int boxIndex) {

  // System.out.println("Find new path for box #" + boxIndex);
  // int listIndex = listIndexes[boxIndex];
  // // temporaryObstacles should be empty before this
  // double bw = problemSpec.getRobotWidth();
  // double scalingFactor = Math.pow(2, listIndex - 1);
  // double gw = bw / scalingFactor;

  // temporaryObstacles.add(pointToObstacle(p1, p2, gw));
  // List<Rectangle2D> tempObs = new ArrayList<>();
  // tempObs.add(temporaryObstacles.get(0));

  // for (int i = 1; i <= numExtensions; i++) {
  // // extend temporary Obstacles
  // scalingFactor = Math.pow(2, i - 1);
  // double margin = (scalingFactor - 1) / scalingFactor * bw;
  // gw = bw / scalingFactor;
  // // should only be one element in the list

  // Rectangle2D fittedObstacle = fittedRects(tempObs, gw, margin).get(0);
  // this.temporaryObstacles.add(fittedObstacle);
  // }

  // for (int i = boxIndex; i < numMovingBoxes; i++) {
  // atGoal[i] = 0;
  // }

  // List<Point2D> newPath = findBoxPath(listIndex, boxIndex);

  // for (int i = boxIndex + 1; i < numMovingBoxes; i++) {
  // atGoal[i] = 1;
  // }
  // // clear temporaryObstacles for next time
  // this.temporaryObstacles = new ArrayList<>();

  // return newPath;
  // }

  public List<List<Point2D>> findAllBoxPaths() {
    List<List<GridNode>> allPaths = new ArrayList<>();

    for (int i = 0; i < numMovingBoxes; i++) {
      int listIndex = listIndexes[i];
      List<GridNode> path = findBoxPath(listIndex, i);

      if (path == null) {
        return null;
      }
      allPaths.add(path);
    }
    List<Integer> order = getOrder();

    if (order == null) {
      return null;
    }

    for (int i = 0; i < order.size(); i++) {
      this.ordering[i] = order.get(i);
      System.out.println(order.get(i));
    }

    return getResultPaths(allPaths);
  }

  private List<List<Point2D>> getResultPaths(List<List<GridNode>> allPaths) {
    List<List<Point2D>> result = new ArrayList<>();
    for (int i = 0; i < ordering.length; i++) {
      List<GridNode> path = allPaths.get(ordering[i]);
      List<Point2D> pointPath = new ArrayList<>();
      for (GridNode g : path) {
        pointPath.add(g.pos);
      }
      result.add(pointPath);
    }
    return result;
  }

  private List<Integer> getOrder() {
    orderObjects.sort(new Comparator<OrderObject>() {
      public int compare(OrderObject a, OrderObject b) {
        if (a.after.contains(b.index))
          return -1;
        if (a.before.contains(b.index))
          return 1;
        if (b.after.contains(a.index))
          return 1;
        if (b.after.contains(a.index))
          return -1;
        return 0;
      }
    });
    List<Integer> result = new ArrayList<>();
    for (OrderObject o : orderObjects) {
      result.add(o.index);
    }
    return result;
  }



  private List<GridNode> findBoxPath(int listIndex, int boxIndex) {
    System.out.println("*** Finding path for box: " + boxIndex + " ***");
    double bw = problemSpec.getRobotWidth();
    boolean pathFound = false;
    double scalingFactor = Math.pow(2, listIndex - 1);
    List<GridNode> path = null;

    while (!pathFound && listIndex < 5) { // todo: is 5 a reasonable number?

      if (listIndex > this.numExtensions) {
        extendObstacles(bw, scalingFactor);
      }
      path = findBoxPathAux(bw, scalingFactor, listIndex, boxIndex);
      if (path != null) {
        pathFound = true;
        double offset = (scalingFactor - 1) * bw / (scalingFactor * 2);
        Util.normalize(path, offset);
        atGoal[boxIndex] = 1;
        listIndexes[boxIndex] = listIndex;
      } else {
        listIndex++;
        scalingFactor = Math.pow(2, listIndex - 1);
        System.out.println("New gridsize: " + bw / scalingFactor);
      }
    }
    if (!pathFound) {
      path = null;
    }
    return path;
  }

  public List<Integer> getIndexList() {
    List<Integer> order = new ArrayList();
    for (int i = 0; i < numMovingBoxes; i++) {
      order.add(this.ordering[i]);
    }
    return order;
  }

  public List<GridNode> findBoxPathAux(double bw, double scalingFactor, int listIndex, int boxIndex) {
    double gw = bw / scalingFactor;
    double offset = (scalingFactor - 1) * gw / 2;
    List<GridNode> startPath = new ArrayList<>();
    List<GridNode> endPath = new ArrayList<>();

    double originalStartX = problemSpec.getMovingBoxes().get(boxIndex).getRect().getCenterX();
    double originalStartY = problemSpec.getMovingBoxes().get(boxIndex).getRect().getCenterY();
    double originalGoalX = problemSpec.getMovingBoxEndPositions().get(boxIndex).getX() + bw / 2;
    double originalGoalY = problemSpec.getMovingBoxEndPositions().get(boxIndex).getY() + bw / 2;

    Point2D shrinkedStart = new Point2D.Double(originalStartX - offset, originalStartY - offset);
    Point2D shrinkedGoal = new Point2D.Double(originalGoalX - offset, originalGoalY - offset);

    shrinkedStart = Util.roundToGrid(shrinkedStart, 0.000001);
    shrinkedGoal = Util.roundToGrid(shrinkedGoal, 0.000001);

    GridNode shrinkedStartNode = new GridNode(this, 0, shrinkedStart, gw, listIndex, boxIndex);
    GridNode shrinkedGoalNode = new GridNode(this, 0, shrinkedGoal, gw, listIndex, boxIndex);

    startPath = pointToGridCenter(shrinkedStartNode, shrinkedGoal, gw, listIndex, boxIndex);
    if (startPath == null) {
      return null;
    }
    endPath = pointToGridCenter(shrinkedGoalNode, shrinkedStart, gw, listIndex, boxIndex);
    if (endPath == null) {
      return null;
    }

    Collections.reverse(endPath);

    Point2D centeredStart = startPath.get(startPath.size() - 1).pos; // get last element in list
    Point2D centeredGoal = endPath.remove(0).pos; // using remove to not get a copy of this point in the path

    Astar agent = new Astar();
    List<GridNode> midPath = (List<GridNode>) agent.search(this, centeredStart, centeredGoal, gw, listIndex, boxIndex);

    if (midPath == null)
      return null;

    List<GridNode> path = new ArrayList<>();
    path.addAll(startPath);
    path.addAll(midPath);
    path.addAll(endPath);

    OrderObject currentBox = new OrderObject(boxIndex);
    for (GridNode g : path) {
      if (g.gridInfo.type == GridType.MOV_BOX_START) {
        int otherBoxIndex = g.gridInfo.boxIndex;
        if (currentBox.after.contains(otherBoxIndex)) {
          System.out.println("start and goal pos in path");
          return null;
        }

        currentBox.before.add(otherBoxIndex);
      }
      if (g.gridInfo.type == GridType.MOV_BOX_GOAL) {
        int otherBoxIndex = g.gridInfo.boxIndex;
        if (currentBox.before.contains(otherBoxIndex)) {
          System.out.println("start and goal pos in path");
          return null;
        }
        currentBox.after.add(otherBoxIndex);
      }
    }
    orderObjects.add(currentBox);
    return path;
  }

  private GridNode closestPoint(List<GridNode> list, Point2D p) {
    double shortestDist = Double.MAX_VALUE;
    double currentDist;
    GridNode closestPoint = null;
    for (GridNode elem : list) {
      if (elem != null) {
        currentDist = Util.manhattanDist(elem.pos, p);
        if (elem.gridInfo.type != GridType.FREE)
          currentDist += 1000;
        if (currentDist < shortestDist) {
          shortestDist = currentDist;
          closestPoint = elem;
        }
      }
    }
    return closestPoint;
  }

  // w should be the width of the grid
  private List<GridNode> pointToGridCenter(GridNode startPoint, Point2D ref, double gw, int listIndex, int boxIndex) {
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
    GridNode startInGrid = closestPoint(gridCenters, startPoint.pos);
    if (startInGrid == null) {
      return null;
    }
    // todo: sometimes the middlePoint is the same as one of the others
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


  /*
   * Extend obstacles to fit the grid w: moving box width
   */
  private void extendObstacles(double bw, double scalingFactor) {
    double margin = (scalingFactor - 1) / scalingFactor * bw;
    double gw = bw / scalingFactor;
    staticObstacles.add(Util.fittedRects(staticObstacles.get(0), gw, margin));
    movingObstacles.add(Util.fittedRects(movingObstacles.get(0), gw, margin));
    boxesStart.add(Util.fittedRects(boxesStart.get(0), gw, margin));
    boxesGoal.add(Util.fittedRects(boxesGoal.get(0), gw, margin));
    numExtensions++;
  }


  private Rectangle2D pointToObstacle(Point2D p1, Point2D p2, double gw) {
    double x1 = p1.getX();
    double y1 = p1.getY();
    double x2 = p2.getX();
    double y2 = p2.getY();
    double xDiff = Math.abs(x1 - x2);
    double yDiff = Math.abs(y1 - y2);
    double obstacleWidth = gw / 4;
    double bottom, left, height, width = 0;

    if (xDiff > yDiff) {
      // line is horizontal ____
      bottom = y1 - obstacleWidth / 2;
      left = Math.min(x1, x2);
      height = obstacleWidth;
      width = gw;
    } else {
      // line is vertical |
      bottom = Math.min(y1, y2);
      left = x1 - obstacleWidth / 2;
      height = gw;
      width = obstacleWidth;
    }
    Rectangle2D tempObstacle = new Rectangle2D.Double(left, bottom, width, height);
    // todo: make a much smaller rectangular obstacle
    return tempObstacle;
  }


  // index i is to know which inner lists to check
  public GridInfo isObstacle(Point2D p, int listIndex, int boxIndex) {
    double bw = problemSpec.getRobotWidth();
    double scalingFactor = Math.pow(2, listIndex - 1);
    double gw = bw / scalingFactor;
    double margin = (scalingFactor - 1) / scalingFactor * bw;

    if (Util.pointOutside(p, bw)) {
      return new GridInfo(GridType.STAT_OBS, -1);
    }
    int index;
    if ((index = Util.isInList(staticObstacles.get(listIndex), p)) > -1) {
      return new GridInfo(GridType.STAT_OBS, index);
    }
    if ((index = Util.isInList(temporaryObstacles, p)) > -1) {
      return new GridInfo(GridType.STAT_OBS, index);
    }
    if ((index = Util.isInList(movingObstacles.get(listIndex), p)) > -1) {
      return new GridInfo(GridType.MOV_OBS, index);
    }
    if ((index = Util.isInList(boxesStart.get(listIndex), p, boxIndex)) > -1) {
      return new GridInfo(GridType.MOV_BOX_START, index);
    }
    if ((index = Util.isInList(boxesGoal.get(listIndex), p, boxIndex)) > -1) {
      return new GridInfo(GridType.MOV_BOX_GOAL, index);
    }
    return new GridInfo(GridType.FREE, -1);
  }


  class OrderObject {
    int index;
    List<Integer> before;
    List<Integer> after;

    OrderObject(int index) {
      this.index = index;
      before = new ArrayList<>();
      after = new ArrayList<>();
    }
  }

}
