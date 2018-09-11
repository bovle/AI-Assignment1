package problem;

import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Test {
    public static void main(String[] args) {
        ProblemSpec ps = new ProblemSpec();
        try {
            ps.loadProblem("input4.txt");
            ps.loadSolution("output.txt");
            List<Rectangle2D> obstacles = new ArrayList<>();
            for (Box box : ps.getMovingBoxPath().get(2304)) {
                obstacles.add(box.getRect());
            }
            for (Box box : ps.getMovingObstacles()) {
                obstacles.add(box.getRect());
            }
            for (StaticObstacle box : ps.getStaticObstacles()) {
                obstacles.add(box.getRect());
            }
            RRT rrt = new RRT();
            rrt.obstacles = obstacles;
            rrt.robotWidth = ps.getRobotWidth();
            rrt.angleStepSize = Math.asin(0.0005 / (ps.getRobotWidth() / 2)) * 2;
            RobotConfig config = ps.getRobotPath().get(3826);
            System.out.println(rrt.collisionFree(config));
            System.out.println(
                    rrt.collisionFreeLine(rrt.directPath(ps.getRobotPath().get(780), ps.getRobotPath().get(799))));

        } catch (IOException e) {
            System.out.println("IO Exception occured");
        }
    }
}