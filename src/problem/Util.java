package problem;

import static tester.Tester.*; // to use MAX_ERROR

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.lang.Math;
import java.util.ArrayList;
import java.util.List;

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
  /*
   * public static void normalize(BoxPath path, double offset) { for (Point2D p :
   * path.startPath) { p.setLocation(p.getX() + offset, p.getY() + offset); } for
   * (Point2D p : path.endPath) { p.setLocation(p.getX() + offset, p.getY() +
   * offset); } for (GridNode g : path.gridPath) { Point2D p = g.pos;
   * p.setLocation(p.getX() + offset, p.getY() + offset); } }
   */

  public static void normalize(List<GridNode> path, double offset) {
    for (GridNode g : path) {
      g.pos.setLocation(Util.roundToStepSize(g.pos.getX() + offset, 0.000001),
          Util.roundToStepSize(g.pos.getY() + offset, 0.000001));
    }

  }

  public static Point2D getGridCenter(Point2D p, double gw) {
    double xIndex = findGridIndex(p.getX(), gw);
    double yIndex = findGridIndex(p.getY(), gw);
    double x = /* gw / 2 + */ xIndex * gw;
    double y = /* gw / 2 + */ yIndex * gw;
    Point2D p1 = new Point2D.Double(x, y);
    Point2D p2 = Util.roundToGrid(p1, 0.000001);
    return p2;
  }

  public static double findGridIndex(double pos, double w) {
    return Math.floor((pos + (w / 2)) / w);
  }

  /*
   * Does this work? The point can be a shrinked box and then it is not the center
   * of the box. TODO: make more general with gridwidth instead of robotWidth so
   * it can be used for moving obstacles as well
   */
  public static boolean pointOutside(Point2D p, double bw, double offset) {
    Rectangle2D box = new Rectangle2D.Double(p.getX() + offset - (bw / 2), p.getY() + offset - (bw / 2), bw, bw);
    Rectangle2D border = new Rectangle2D.Double(0, 0, 1, 1);
    border = Util.grow(border, MAX_ERROR);
    return !border.contains(box);
  }

  public static List<Rectangle2D> fittedRects(List<Rectangle2D> originalRects, double gw, double margin) {
    List<Rectangle2D> fittedRects = new ArrayList<>();
    for (Rectangle2D original : originalRects) {
      Rectangle2D fittedRect = fitToGrid(original, gw, margin);
      fittedRects.add(fittedRect);
    }
    return fittedRects;
  }

  public static Rectangle2D fitToGrid(Rectangle2D original, double gw, double margin) {
    // first: resize eveything
    double newX = original.getX() - margin;
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

  public static Point2D roundToGrid(Point2D p, double gridWidth) {
    double roundedX = roundToStepSize(p.getX(), gridWidth);
    double roundedY = roundToStepSize(p.getY(), gridWidth);
    return new Point2D.Double(roundedX, roundedY);
  }

  public static double roundToStepSize(double d, double stepSize) {
    double factor = 1 / stepSize;
    return Math.round(d * factor) / factor;
  }

  public static int isInList(List<Rectangle2D> list, Point2D p, int currentIndex) {
    for (int i = 0; i < list.size(); i++) {
      if (i != currentIndex) {
        if (list.get(i).contains(p.getX(), p.getY())) {
          return i;

        }
      }
    }
    return -1;
  }

  public static int isInList(List<Rectangle2D> list, Point2D p) {
    for (int i = 0; i < list.size(); i++) {
      if (list.get(i).contains(p.getX(), p.getY())) {
        return i;
      }
    }
    return -1;
  }
}
