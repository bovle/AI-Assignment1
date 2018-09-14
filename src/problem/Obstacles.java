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

  public Obstacles(MovingBoxPlanner boxPlanner, Integer obstacleIndex, List<List<Rectangle2D>> boxPaths) {
    this.staticObstacles = boxPlanner.getStaticObstacles();
    this.movingObstacles = boxPlanner.getMovingObstacles();
    this.boxesStart = boxPlanner.getBoxesStart();
    this.boxPaths = boxPaths;
    this.obstacleIndex = obstacleIndex;
    this.ow = movingObstacles.get(0).get(obstacleIndex).getWidth();
    this.numExtensions = 0;
  }


  public void updateObstacle(Point2D p) {
    Rectangle2D newObstacle = Util.pointToRect(p, ow);
    this.movingObstacles.get(0).remove(obstacleIndex);
    this.movingObstacles.get(0).add(obstacleIndex, newObstacle);
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

  public GridInfo isObstacle(Point2D p, int listIndex, int boxIndex) {
    double scalingFactor = Math.pow(2, listIndex - 1);
    double gw = ow / scalingFactor;
    double margin = (scalingFactor - 1) / scalingFactor * ow;

    if (Util.oldPointOutside(p, margin, gw)) {
      return new GridInfo(GridType.STAT_OBS, -1);
    }
    int index;
    if ((index = Util.isInList(staticObstacles.get(listIndex), p)) > -1) {
      return new GridInfo(GridType.STAT_OBS, index);
    }

    if ((index = Util.isInList(movingObstacles.get(listIndex), p, obstacleIndex)) > -1) { // added obstacleIndex to ignore itself
      return new GridInfo(GridType.MOV_OBS, index);
    }
    if ((index = Util.isInList(boxesStart.get(listIndex), p)) > -1) { // removed boxIndex to consider all start positions
      return new GridInfo(GridType.MOV_BOX_START, index);
    }

    if ((index = Util.isInList(boxPaths.get(listIndex), p)) > -1) {
      return new GridInfo(GridType.MOV_BOX_PATH, index);
    }
    return new GridInfo(GridType.FREE, -1);
  }

}
