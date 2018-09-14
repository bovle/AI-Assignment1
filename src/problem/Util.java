package problem;

import static tester.Tester.*; // to use MAX_ERROR

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
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

  public static double manhattanDist(Point2D a, Point2D b) {
    return Math.abs(a.getX() - b.getX()) + Math.abs(a.getY() - b.getY());
  }

  public static Point2D translateOneStep(Point2D position, double angle) {
    double nextX = 0.001 * Math.cos(angle) + position.getX();
    double nextY = 0.001 * Math.sin(angle) + position.getY();
    return new Point2D.Double(nextX, nextY);
  }

  public static Rectangle2D pointToRect(Point2D pos, double width) {
    return new Rectangle2D.Double(pos.getX() - (width / 2), pos.getY() - (width / 2), width, width);
  }

  public static Point2D rectToPoint(Rectangle2D rect) {
    return new Point2D.Double(rect.getCenterX(), rect.getCenterY());
  }

  public static Rectangle2D grow(Rectangle2D rect, double delta) {
    return new Rectangle2D.Double(rect.getX() - delta, rect.getY() - delta, rect.getWidth() + 2 * delta,
        rect.getHeight() + 2 * delta);
  }

  public static Point2D roundToGrid(Point2D p, double gridWidth) {
    double roundedX = roundToStepSize(p.getX(), gridWidth);
    double roundedY = roundToStepSize(p.getY(), gridWidth);
    return new Point2D.Double(roundedX, roundedY);
  }

  public static double roundToStepSize(double d, double stepSize) {
    double factor = 1 / stepSize;
    return Math.round(d * factor) / factor;
  }
}
