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

public class MovingBoxPlanner {
  private ProblemSpec problemSpec;
  // private List<Point2D> currentStaticObstacles;
  private List<List<Rectangle2D>> staticObstacles;
  private List<List<Rectangle2D>> movingObstacles;
  private List<List<Rectangle2D>> boxesStart;
  private List<List<Rectangle2D>> boxesGoal;
  private Integer numMovingBoxes;
  private Integer[] atGoal;
  private Integer[] ordering; // the order the boxes are going to move
  private Integer[] listIndexes; // the index where the algorithm finds a path
  private Integer numExtensions;
  private List<Rectangle2D> temporaryObstacles;

  public MovingBoxPlanner(ProblemSpec ps) {
    this.problemSpec = ps;
    this.ordering = new Integer[ps.getNumMovingBoxes()];
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

    this.totalOrdering = new ArrayList<>();

    // temporaryObstacles
    this.temporaryObstacles = new ArrayList<>();

    this.numExtensions = 0;

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
    List<BoxPath> allPaths = new ArrayList<>();

    for (int i = 0; i < numMovingBoxes; i++) {
      int listIndex = listIndexes[i];
      BoxPath path = findBoxPath(listIndex, i);

      if (path == null) {
        return null;
      }
      allPaths.add(path);
    }
    List<Integer> order = getOrder(allPaths);

    if (order == null) {
      return null;
    }

    for (int i = 0; i < order.size(); i++) {
      this.ordering[i] = order.get(i);
      System.out.println(order.get(i));
    }

    return getResultPaths(allPaths);
  }

  private List<List<Point2D>> getResultPaths(List<BoxPath> allPaths) {
    List<List<Point2D>> result = new ArrayList<>();
    for (int i = 0; i < ordering.length; i++) {
      BoxPath path = allPaths.get(ordering[i]);
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

  private List<Integer> getOrder(List<BoxPath> allPaths) {
    class orderObject {
      int index;
      List<Integer> before;
      List<Integer> after;

      orderObject(int index) {
        this.index = index;
        before = new ArrayList<>();
        after = new ArrayList<>();
      }
    }
    List<orderObject> order = new LinkedList<>();
    for (int i = 0; i < numMovingBoxes; i++) {
      orderObject currentBox = new orderObject(i);
      List<GridNode> gridPath = allPaths.get(i).gridPath;
      for (GridNode g : gridPath) {
        if (g.gridInfo.type == GridType.MOV_BOX_START) {
          int otherBoxIndex = g.gridInfo.boxIndex;
          if (currentBox.after.contains(otherBoxIndex))
            return null;
          currentBox.before.add(otherBoxIndex);
        }
        if (g.gridInfo.type == GridType.MOV_BOX_GOAL) {
          int otherBoxIndex = g.gridInfo.boxIndex;
          if (currentBox.before.contains(otherBoxIndex))
            return null;
          currentBox.after.add(otherBoxIndex);
        }
      }
      order.add(currentBox);
    }
    order.sort(new Comparator<orderObject>() {
      public int compare(orderObject a, orderObject b) {
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
    for (orderObject o : order) {
      result.add(o.index);
    }
    return result;
  }

  private void normalize(BoxPath path, double offset) {
    for (Point2D p : path.startPath) {
      p.setLocation(p.getX() + offset, p.getY() + offset);
    }
    for (Point2D p : path.endPath) {
      p.setLocation(p.getX() + offset, p.getY() + offset);
    }
    for (GridNode g : path.gridPath) {
      Point2D p = g.pos;
      p.setLocation(p.getX() + offset, p.getY() + offset);
    }
  }

  private BoxPath findBoxPath(int listIndex, int boxIndex) {
    System.out.println("*** Finding path for box: " + boxIndex + " ***");
    double bw = problemSpec.getRobotWidth();
    boolean pathFound = false;
    double scalingFactor = Math.pow(2, listIndex - 1);
    BoxPath path = null;

    while (!pathFound && listIndex < 5) { // todo: is 5 a reasonable number?

      if (listIndex > this.numExtensions) {
        extendObstacles(bw, scalingFactor);
      }
      path = findBoxPathAux(bw, scalingFactor, listIndex, boxIndex);
      if (path != null) {
        pathFound = true;
        double offset = (scalingFactor - 1) * bw / (scalingFactor * 2);
        normalize(path, offset);
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

  public BoxPath findBoxPathAux(double bw, double scalingFactor, int listIndex, int boxIndex) {
    double gw = bw / scalingFactor;
    double offset = (scalingFactor - 1) * gw / 2;
    List<Point2D> startPath = new ArrayList<>();
    List<Point2D> endPath = new ArrayList<>();

    double originalStartX = problemSpec.getMovingBoxes().get(boxIndex).getRect().getCenterX();
    double originalStartY = problemSpec.getMovingBoxes().get(boxIndex).getRect().getCenterY();
    double originalGoalX = problemSpec.getMovingBoxEndPositions().get(boxIndex).getX() + bw / 2;
    double originalGoalY = problemSpec.getMovingBoxEndPositions().get(boxIndex).getY() + bw / 2;

    Point2D shrinkedStart = new Point2D.Double(originalStartX - offset, originalStartY - offset);
    Point2D shrinkedGoal = new Point2D.Double(originalGoalX - offset, originalGoalY - offset);

    startPath = pointToGridCenter(shrinkedStart, shrinkedGoal, gw, listIndex, boxIndex);
    if (startPath == null) {
      return null;
    }
    endPath = pointToGridCenter(shrinkedGoal, shrinkedStart, gw, listIndex, boxIndex);
    if (endPath == null) {
      return null;
    }

    Collections.reverse(endPath);

    Point2D centeredStart = startPath.get(startPath.size() - 1); // get last element in list
    Point2D centeredGoal = endPath.remove(0); // using remove to not get a copy of this point in the path

    Astar agent = new Astar();
    List<GridNode> path = (List<GridNode>) agent.search(this, centeredStart, centeredGoal, gw, listIndex, boxIndex);
    if (path == null)
      return null;
    return new BoxPath(path, startPath, endPath);
  }

  private Point2D closestPoint(List<Point2D> list, Point2D p) {
    double shortestDist = 2; // maximal manhattan distance is 2
    double currentDist;
    Point2D closestPoint = null;
    for (Point2D elem : list) {
      if (elem != null) {
        currentDist = Util.manhattanDist(elem, p);
        if (currentDist < shortestDist) {
          shortestDist = currentDist;
          closestPoint = elem;
        }
      }
    }
    return closestPoint;
  }

  // w should be the width of the grid
  private List<Point2D> pointToGridCenter(Point2D point, Point2D ref, double gw, int listIndex, int boxIndex) {
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
      currentCenter = getGridCenter(p, gw);
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
    // todo: choose center closest to goal?
    Point2D startInGrid = closestPoint(gridCenters, ref);
    // todo: sometimes the middlePoint is the same as one of the others
    Point2D middlePoint = new Point2D.Double(x, startInGrid.getY());
    List<Point2D> path = new ArrayList<>();
    path.add(point);
    if (!Util.equalPositions(point, middlePoint)) {
      path.add(middlePoint);
    }
    return path;
  }

  private Point2D getGridCenter(Point2D p, double gw) {
    double xIndex = findGridIndex(p.getX(), gw);
    double yIndex = findGridIndex(p.getY(), gw);
    double x = gw / 2 + xIndex * gw;
    double y = gw / 2 + yIndex * gw;
    return new Point2D.Double(x, y);
  }

  /*
   * Extend obstacles to fit the grid w: moving box width
   */
  private void extendObstacles(double bw, double scalingFactor) {
    double margin = (scalingFactor - 1) / scalingFactor * bw;
    double gw = bw / scalingFactor;
    staticObstacles.add(fittedRects(staticObstacles.get(0), gw, margin));
    movingObstacles.add(fittedRects(movingObstacles.get(0), gw, margin));
    boxesStart.add(fittedRects(boxesStart.get(0), gw, margin));
    boxesGoal.add(fittedRects(boxesGoal.get(0), gw, margin));
    numExtensions++;
  }

  private List<Rectangle2D> fittedRects(List<Rectangle2D> originalRects, double gw, double margin) {
    List<Rectangle2D> fittedRects = new ArrayList<>();
    for (Rectangle2D original : originalRects) {
      Rectangle2D fittedRect = fitToGrid(original, gw, margin);
      fittedRects.add(fittedRect);
    }
    return fittedRects;
  }

  private Rectangle2D fitToGrid(Rectangle2D original, double gw, double margin) {
    // first: resize eveything
    double newX = original.getX() - margin; // todo: should it also be -w/2?
    double newY = original.getY() - margin;
    double newWidth = original.getWidth() + margin;
    double newHeigth = original.getHeight() + margin;

    // second: make it fit to current grid
    double leftIndex = findGridIndex(newX, gw);
    double rightIndex = findGridIndex(newX + newWidth, gw) + 1;
    double bottomIndex = findGridIndex(newY, gw);
    double topIndex = findGridIndex(newY + newHeigth, gw) + 1;
    double fittedWidth = (rightIndex - leftIndex) * gw;
    double fittedHeight = (topIndex - bottomIndex) * gw;
    Rectangle2D fittedRect = new Rectangle2D.Double(leftIndex * gw, bottomIndex * gw, fittedWidth, fittedHeight);
    return fittedRect;
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

  private double findGridIndex(double pos, double w) {
    return Math.floor(pos / w);
  }

  private boolean pointOutside(Point2D p, double margin, double gridWidth) {
    double x = p.getX();
    double y = p.getY();
    return (x < gridWidth / 2 || x > 1 - margin - (gridWidth / 2) || y < gridWidth / 2
        || y > 1 - margin - (gridWidth / 2));
  }

  // index i is to know which inner lists to check
  public GridInfo isObstacle(Point2D p, int listIndex, int boxIndex) {
    double bw = problemSpec.getRobotWidth();
    double scalingFactor = Math.pow(2, listIndex - 1);
    double gw = bw / scalingFactor;
    double margin = (scalingFactor - 1) / scalingFactor * bw;

    if (pointOutside(p, margin, gw)) {
      return new GridInfo(GridType.STAT_OBS, -1);
    }
    int index;
    if ((index = isInList(staticObstacles.get(listIndex), p)) > -1) {
      return new GridInfo(GridType.STAT_OBS, index);
    }
    if ((index = isInList(temporaryObstacles, p)) > -1) {
      return new GridInfo(GridType.STAT_OBS, index);
    }
    if ((index = isInList(movingObstacles.get(listIndex), p)) > -1) {
      return new GridInfo(GridType.MOV_OBS, index);
    }
    if ((index = isInList(boxesStart.get(listIndex), p, boxIndex)) > -1) {
      return new GridInfo(GridType.MOV_BOX_START, index);
    }
    if ((index = isInList(boxesGoal.get(listIndex), p, boxIndex)) > -1) {
      return new GridInfo(GridType.MOV_BOX_GOAL, index);
    }
    return new GridInfo(GridType.FREE, -1);
  }

  private int isInList(List<Rectangle2D> list, Point2D p, int currentIndex) {
    for (int i = 0; i < list.size(); i++) {
      if (i != currentIndex) {
        if (list.get(i).contains(p.getX(), p.getY())) {
          return i;

        }
      }
    }
    return -1;
  }

  private int isInList(List<Rectangle2D> list, Point2D p) {
    for (int i = 0; i < list.size(); i++) {
      if (list.get(i).contains(p.getX(), p.getY())) {
        return i;
      }
    }
    return -1;
  }

}
