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

public class Obstacles {
  private double ow;
  private Integer obstacleIndex;
  private List<List<Rectangle2D>> staticObstacles;
  private List<List<Rectangle2D>> movingObstacles;
  private List<List<Rectangle2D>> boxesStart;
  private List<List<Rectangle2D>> boxPaths;
  public Integer numExtensions;

  public Obstacles(MovingBoxPlanner boxPlanner, Integer obstacleIndex, List<List<Rectangle2D>> bP) {
    this.staticObstacles = new ArrayList<>();
    this.movingObstacles = new ArrayList<>();
    this.boxesStart = new ArrayList<>();
    this.boxPaths = new ArrayList<>();

    this.staticObstacles.add(boxPlanner.getStaticObstacles());
    this.movingObstacles.add(boxPlanner.getMovingObstacles());
    this.boxesStart.add(boxPlanner.getBoxesStart());

    List<Rectangle2D> allPaths = new ArrayList<>();
    for (List<Rectangle2D> path : bP) {
      allPaths.addAll(path);
    }
    this.boxPaths.add(allPaths);
    this.obstacleIndex = obstacleIndex;
    this.ow = Util.roundToStepSize(this.movingObstacles.get(0).get(obstacleIndex).getWidth() - 2 * MAX_ERROR, 0.000001);
    this.numExtensions = 0;
  }

  public void updateObstacle(Point2D p, int changedIndex) {
    Rectangle2D newObstacle = Util.pointToRect(p, ow);
    newObstacle = Util.grow(newObstacle, MAX_ERROR);
    this.movingObstacles.get(0).remove(changedIndex);
    this.movingObstacles.get(0).add(changedIndex, newObstacle);
  }

  public void extendObstacles(double ow, double scalingFactor) {
    double margin = (scalingFactor - 1) / scalingFactor * ow;
    double gw = ow / scalingFactor;
    staticObstacles.add(Util.fittedRects(staticObstacles.get(0), gw, margin));
    movingObstacles.add(Util.fittedRects(movingObstacles.get(0), gw, margin));
    boxesStart.add(Util.fittedRects(boxesStart.get(0), gw, margin));
    boxPaths.add(Util.fittedRects(boxPaths.get(0), gw, margin));
    numExtensions++;
  }

  public GridInfo isObstacle(Point2D p, int listIndex, int obstacleIndex) {
    double scalingFactor = Math.pow(2, listIndex - 1);
    double gw = ow / scalingFactor;
    double offset = (scalingFactor - 1) * ow / (scalingFactor * 2);

    if (Util.pointOutside(p, ow, offset)) {
      return new GridInfo(GridType.STAT_OBS, -1);
    }
    int index;
    if ((index = Util.isInList(staticObstacles.get(listIndex), p)) > -1) {
      return new GridInfo(GridType.STAT_OBS, index);
    }

    if ((index = Util.isInList(movingObstacles.get(listIndex), p, obstacleIndex)) > -1) { // added obstacleIndex to
                                                                                          // ignore itself
      return new GridInfo(GridType.MOV_OBS, index);
    }
    if ((index = Util.isInList(boxesStart.get(listIndex), p)) > -1) { // removed boxIndex to consider all start
                                                                      // positions
      return new GridInfo(GridType.MOV_BOX_START, index);
    }

    if ((index = Util.isInList(boxPaths.get(listIndex), p)) > -1) {
      return new GridInfo(GridType.MOV_BOX_PATH, index);
    }
    return new GridInfo(GridType.FREE, -1);
  }

}
