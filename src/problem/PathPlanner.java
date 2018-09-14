package problem;
import java.awt.geom.Point2D;

public interface PathPlanner {
    public GridInfo isObstacle(Point2D p, int listIndex, int boxIndex);
}
