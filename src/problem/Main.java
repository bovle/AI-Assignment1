package problem;

import java.awt.geom.Rectangle2D;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import tester.Tester;

public class Main {
    public static void main(String[] args) {
        ProblemSpec ps = new ProblemSpec();
        try {
            ps.loadProblem("input2.txt");
            List<Rectangle2D> obstacles = new ArrayList<>();
            for (Box box : ps.getMovingBoxes()) {
                obstacles.add(box.getRect());
            }
            for (Box box : ps.getMovingObstacles()) {
                obstacles.add(box.getRect());
            }
            for (StaticObstacle box : ps.getStaticObstacles()) {
                obstacles.add(box.getRect());
            }
            RRT rrt = new RRT();
            List<RobotConfig> path = rrt.calculatePath(ps.getInitialRobotConfig(),
                    new RobotConfig(new double[] { 0.2, 0.0499 }, 0.0), ps.getRobotWidth(), obstacles);
            outputPath(path);

        } catch (IOException e) {
            System.out.println("IO Exception occured");
        }
        System.out.println("Finished loading!");

    }

    public static void outputPath(List<RobotConfig> path) throws IOException {
        FileWriter fw = new FileWriter("output.txt");
        PrintWriter pw = new PrintWriter(fw);
        pw.println(path.size());
        for (int i = 0; i < path.size(); i++) {
            double x = path.get(i).getPos().getX();
            double y = path.get(i).getPos().getY();
            double angle = path.get(i).getOrientation();
            pw.println(x + " " + y + " " + angle + " 0.2 0.1 0.35 0.1 0.09 0.05");
        }
        pw.close();
    }
}