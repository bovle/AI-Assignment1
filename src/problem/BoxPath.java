package problem;

import java.awt.geom.Point2D;
import java.util.List;

public class BoxPath {
    public List<GridNode> gridPath;
    public List<Point2D> startPath;
    public List<Point2D> endPath;

    public BoxPath(List<GridNode> gridPath, List<Point2D> startPath, List<Point2D> endPath) {
        this.gridPath = gridPath;
        this.startPath = startPath;
        this.endPath = endPath;
    }

}