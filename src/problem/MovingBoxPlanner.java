package problem;

import static tester.Tester.*; // to use MAX_ERROR

import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collections;
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

    setOrder();
    this.numExtensions = 0;

  }

  // p1 and p2 are the end points of the robot line
  public List<Point2D> findNewPath(Point2D p1, Point2D p2, int boxIndex) {

    System.out.println("Find new path for box #" + boxIndex);
    int listIndex = listIndexes[boxIndex];
    // temporaryObstacles should be empty before this
    double bw = problemSpec.getRobotWidth();
    double scalingFactor = Math.pow(2, listIndex - 1);
    double gw = bw / scalingFactor;

    temporaryObstacles.add(pointToObstacle(p1, p2, gw));
    List<Rectangle2D> tempObs = new ArrayList<>();
    tempObs.add(temporaryObstacles.get(0));

    for (int i = 1; i <= numExtensions; i++) {
      // extend temporary Obstacles
      scalingFactor = Math.pow(2, i - 1);
      double margin = (scalingFactor - 1) / scalingFactor * bw;
      gw = bw / scalingFactor;
      // should only be one element in the list

      Rectangle2D fittedObstacle = fittedRects(tempObs, gw, margin).get(0);
      this.temporaryObstacles.add(fittedObstacle);
    }

    for (int i = boxIndex; i < numMovingBoxes; i++) {
      atGoal[i] = 0;
    }

    List<Point2D> newPath = findBoxPath(listIndex, boxIndex);

    for (int i = boxIndex + 1; i < numMovingBoxes; i++) {
      atGoal[i] = 1;
    }
    // clear temporaryObstacles for next time
    this.temporaryObstacles = new ArrayList<>();

    return newPath;
  }

  public List<List<Point2D>> findAllBoxPaths() {
    List<List<Point2D>> allPaths = new ArrayList<>();

    for (int i = 0; i < numMovingBoxes; i++) {
      int boxIndex = ordering[i];
      int listIndex = listIndexes[boxIndex];
      List<Point2D> path = findBoxPath(listIndex, boxIndex);

      if (path == null) {
        return null;
      }
      allPaths.add(path);
    }
    return allPaths;
  }

  private void normalize(List<Point2D> path, double offset) {
    for (Point2D p : path) {
      p.setLocation(p.getX() + offset, p.getY() + offset);
    }
  }

  private List<Point2D> findBoxPath(int listIndex, int boxIndex) {
    System.out.println("*** Finding path for box: " + boxIndex + " ***");
    double bw = problemSpec.getRobotWidth();
    boolean pathFound = false;
    double scalingFactor = Math.pow(2, listIndex - 1);
    List<Point2D> path = new ArrayList<>();

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
        scalingFactor *= 2;
        listIndex++;
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

  public List<Point2D> findBoxPathAux(double bw, double scalingFactor, int listIndex, int boxIndex) {
    double gw = bw / scalingFactor;
    double offset = (scalingFactor - 1) * gw / 2;
    List<Point2D> fullPath = new ArrayList<>();
    List<Point2D> startPath = new ArrayList<>();
    List<Point2D> endPath = new ArrayList<>();

    double originalStartX = problemSpec.getMovingBoxes().get(boxIndex).getRect().getCenterX();
    double originalStartY = problemSpec.getMovingBoxes().get(boxIndex).getRect().getCenterY();
    double originalGoalX = problemSpec.getMovingBoxEndPositions().get(boxIndex).getX() + bw / 2;
    double originalGoalY = problemSpec.getMovingBoxEndPositions().get(boxIndex).getY() + bw / 2;

    Point2D shrinkedStart = new Point2D.Double(originalStartX - offset, originalStartY - offset);
    Point2D shrinkedGoal = new Point2D.Double(originalGoalX - offset, originalGoalY - offset);

    startPath = pointToGridCenter(shrinkedStart, shrinkedGoal, gw, listIndex, boxIndex);
    endPath = pointToGridCenter(shrinkedGoal, shrinkedStart, gw, listIndex, boxIndex);

    Collections.reverse(endPath);

    Point2D centeredStart = startPath.get(startPath.size() - 1); // get last element in list
    Point2D centeredGoal = endPath.remove(0); // using remove to not get a copy of this point in the path

    fullPath.addAll(startPath);

    Astar agent = new Astar();
    List<GridNode> path = (List<GridNode>) agent.search(this, centeredStart, centeredGoal, gw, listIndex, boxIndex);
    if (path == null)
      return null;

    for (GridNode g : path) {
      fullPath.add(g.pos);
    }

    fullPath.addAll(endPath);
    return fullPath;
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
    for (Point2D p : corners) {
      currentCenter = getGridCenter(p, gw);
      if (GridType.FREE == isObstacle(currentCenter, listIndex, boxIndex)) {
        gridCenters.add(currentCenter);
      } else {
        gridCenters.add(null);
      }
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

  private void setOrder() {
    this.ordering = new Integer[numMovingBoxes];
    // simple ordering now: just as indexed in input file
    for (int i = 0; i < numMovingBoxes; i++) {
      this.ordering[i] = i;
    }
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

  private boolean pointOutside(Point2D p, double margin) {
    double x = p.getX();
    double y = p.getY();
    return (x < 0 || x > 1 - margin || y < 0 || y > 1 - margin);
  }

  // index i is to know which inner lists to check
  public GridType isObstacle(Point2D p, int listIndex, int boxIndex) {
    double bw = problemSpec.getRobotWidth();
    double scalingFactor = Math.pow(2, listIndex - 1);
    double margin = (scalingFactor - 1) / scalingFactor * bw;

    if (pointOutside(p, margin)) {
      return GridType.STAT_OBS;
    }
    if (isInList(staticObstacles.get(listIndex), p)) {
      return GridType.STAT_OBS;
    }
    if (isInList(temporaryObstacles, p)) {
      return GridType.STAT_OBS;
    }
    if (isInList(movingObstacles.get(listIndex), p)) {
      return GridType.MOV_OBS;
    }
    if (startOrGoal(p, listIndex, boxIndex)) {
      return GridType.MOV_BOX;
    }
    return GridType.FREE;
  }

  private boolean startOrGoal(Point2D p, int listIndex, int boxIndex) {
    List<Rectangle2D> starts = boxesStart.get(listIndex);
    List<Rectangle2D> goals = boxesGoal.get(listIndex);
    Rectangle2D currentRect;
    for (int i = 0; i < starts.size(); i++) {
      if (i != boxIndex) {
        if (atGoal[i] == 1) {
          currentRect = goals.get(i);
        } else {
          currentRect = starts.get(i);
        }
        if (currentRect.contains(p.getX(), p.getY())) {
          return true;

        }
      }
    }
    return false;
  }

  private boolean isInList(List<Rectangle2D> list, Point2D p) {
    for (Rectangle2D r : list) {
      if (r.contains(p.getX(), p.getY())) {
        return true;
      }

    }
    return false;
  }

}
