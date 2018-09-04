package problem;
import static tester.Tester.*; // to use MAX_ERROR

import java.awt.geom.Point2D;
import java.lang.Math;

public class Util {

  public static Boolean equalPositions(Point2D pos1, Point2D pos2) {
    double xDiff = Math.abs(pos1.getX() - pos2.getX());
    double yDiff = Math.abs(pos1.getY() - pos2.getY());
    return (validDiff(xDiff) && validDiff(yDiff));
  }

  private static Boolean validDiff(double diff) {
    return (Math.abs(diff) < MAX_ERROR);
  }

  public static GridType findGridType(Point2D p) {
    // todo1: return STAT_OBS for walls/staticObstacles
    return GridType.FREE;
  }
}
