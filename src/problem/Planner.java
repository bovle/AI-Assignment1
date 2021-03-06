package problem;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class Planner {

    ProblemSpec ps;
    RRT rrt;
    MovingBoxPlanner boxPlanner;
    List<RobotConfig> robotPath;
    List<List<Point2D>> movingBoxPaths; // row = stepnr | column = boxIndex

    public Planner(ProblemSpec ps) {
        this.ps = ps;
        rrt = new RRT();
        boxPlanner = new MovingBoxPlanner(ps);
    }

    public void plan(String outputFile) throws IOException {
        robotPath = new ArrayList<>();
        movingBoxPaths = new ArrayList<>();
        movingBoxPaths.add(new ArrayList());
        List<Rectangle2D> obstacles = new ArrayList<>();
        for (Box box : ps.getMovingBoxes()) {
            obstacles.add(box.getRect());
            movingBoxPaths.get(0).add(new Point2D.Double(box.getRect().getCenterX(), box.getRect().getCenterY()));
        }
        for (Box box : ps.getMovingObstacles()) {
            obstacles.add(box.getRect());
            movingBoxPaths.get(0).add(new Point2D.Double(box.getRect().getCenterX(), box.getRect().getCenterY()));
        }
        for (StaticObstacle box : ps.getStaticObstacles()) {
            obstacles.add(box.getRect());
        }

        List<List<Point2D>> boxPaths = boxPlanner.findAllBoxPaths();
        if (boxPaths == null) {
            System.err.println("no solution");
            return;
        }
        List<Integer> indexList = boxPlanner.getIndexList();

        MovingObstaclePlanner mop = new MovingObstaclePlanner(ps, boxPlanner, boxPaths);
        List<List<Point2D>> obstaclePaths = mop.planAllPaths();
        if (obstaclePaths == null) {
            System.err.println("no solution");
            return;
        }

        RobotConfig currentConfig = ps.getInitialRobotConfig();
        for (int i = 0; i < obstaclePaths.size(); i++) {
            List<Point2D> obstaclePath = obstaclePaths.get(i);
            if (obstaclePath.size() < 2)
                continue;

            System.out.println(" *** " + i + " *** ");
            obstaclePath.forEach(p -> System.out.println(p));

            Rectangle2D movingRect = obstacles.remove(ps.getNumMovingBoxes() + i);

            RobotConfig failConfig = calcPaths(currentConfig, ps.getNumMovingBoxes() + i, obstaclePath, obstacles,
                    movingRect.getWidth());
            if (failConfig != null) {
                System.out.println("failed at: " + failConfig.getPos() + " | " + failConfig.getOrientation());
                movingBoxPaths.remove(0);
                outputPath(outputFile);
                System.err.println("no solution");
                return;
            }
            obstacles.add(ps.getNumMovingBoxes() + i,
                    Util.pointToRect(obstaclePath.get(obstaclePath.size() - 1), movingRect.getWidth()));
            currentConfig = robotPath.get(robotPath.size() - 1);
        }

        for (int i = 0; i < boxPaths.size(); i++) {
            List<Point2D> path = boxPaths.get(i);
            // System.out.println(" *** " + i + " *** ");
            // path.forEach(p -> System.out.println(p));
            int boxIndex = indexList.get(i);
            Rectangle2D movingRect = obstacles.remove(boxIndex);

            RobotConfig failConfig = calcPaths(currentConfig, boxIndex, path, obstacles, movingRect.getWidth());
            if (failConfig != null) {
                System.out.println("failed at: " + failConfig.getPos() + " | " + failConfig.getOrientation());
                movingBoxPaths.remove(0);
                outputPath(outputFile);
                System.err.println("no solution");
                return;
            }
            obstacles.add(boxIndex, Util.pointToRect(path.get(path.size() - 1), movingRect.getWidth()));
            currentConfig = robotPath.get(robotPath.size() - 1);
        }

        movingBoxPaths.remove(0);

        outputPath(outputFile);
    }

    private RobotConfig calcPaths(RobotConfig initConfig, int boxIndex, List<Point2D> boxPath,
            List<Rectangle2D> staticObstacles, double boxWidth) {
        List<DirectionalLine> lines = splitByDirection(boxPath);
        RobotConfig currentConfig = initConfig;
        double robotWidth = ps.getRobotWidth();
        List<RobotConfig> tempRobotPath = new ArrayList<>();
        List<List<Point2D>> tempMovingBoxPaths = new ArrayList<>();
        tempMovingBoxPaths.add(new ArrayList<>());
        for (int i = 0; i < ps.getMovingBoxes().size() + ps.getMovingObstacles().size(); i++) {
            tempMovingBoxPaths.get(0).add(movingBoxPaths.get(movingBoxPaths.size() - 1).get(i));
        }
        for (int i = 0; i < lines.size(); i++) {
            DirectionalLine line = lines.get(i);
            RobotConfig nextConfig = robotConfigFromBoxPos(line.startPosition, line.direction, boxWidth);
            List<Rectangle2D> obstacles = new ArrayList<>();
            obstacles.addAll(staticObstacles);
            obstacles.add(Util.pointToRect(line.startPosition, boxWidth));
            List<RobotConfig> pathToStart = rrt.calculatePath(currentConfig, nextConfig, robotWidth, obstacles);
            if (pathToStart == null) {
                // ### uncomment to test in visualizer ###
                robotPath.addAll(tempRobotPath);
                movingBoxPaths.addAll(tempMovingBoxPaths);
                return nextConfig;
            }
            tempRobotPath.addAll(pathToStart);

            while (tempMovingBoxPaths.size() < tempRobotPath.size()) {
                extendBoxPaths(tempMovingBoxPaths);
            }
            currentConfig = nextConfig;
            nextConfig = robotConfigFromBoxPos(line.endPosition, line.direction, boxWidth);
            addDirectionalLineToPaths(tempRobotPath, tempMovingBoxPaths, boxIndex, line.direction, currentConfig,
                    nextConfig, boxWidth);
            currentConfig = nextConfig;
        }
        robotPath.addAll(tempRobotPath);

        movingBoxPaths.addAll(tempMovingBoxPaths);

        return null;

    }

    private void addDirectionalLineToPaths(List<RobotConfig> tempRobotPath, List<List<Point2D>> tempMovingBoxPaths,
            int boxIndex, double movementDirection, RobotConfig startConfig, RobotConfig goalConfig, double boxWidth) {
        double distance = startConfig.getPos().distance(goalConfig.getPos());
        int steps = (int) Math.floor(distance / 0.001);
        RobotConfig currentConfig = startConfig;
        tempRobotPath.add(startConfig);
        extendBoxPaths(tempMovingBoxPaths);
        for (int i = 0; i < steps; i++) {
            currentConfig = new RobotConfig(Util.translateOneStep(currentConfig.getPos(), movementDirection),
                    currentConfig.getOrientation());
            tempRobotPath.add(currentConfig);
            extendBoxPaths(tempMovingBoxPaths, boxIndex, currentConfig, movementDirection, boxWidth);
        }
        if (!currentConfig.getPos().equals(goalConfig.getPos())) {
            tempRobotPath.add(goalConfig);
            extendBoxPaths(tempMovingBoxPaths, boxIndex, goalConfig, movementDirection, boxWidth);
        }
    }

    private List<DirectionalLine> splitByDirection(List<Point2D> boxPath) {
        List<DirectionalLine> lines = new ArrayList<>();
        Point2D startPosition = boxPath.get(0);
        double y = boxPath.get(1).getY() - boxPath.get(0).getY();
        double x = boxPath.get(1).getX() - boxPath.get(0).getX();
        double currentDirection = Math.atan2(y, x);

        for (int i = 2; i < boxPath.size(); i++) {
            y = boxPath.get(i).getY() - boxPath.get(i - 1).getY();
            x = boxPath.get(i).getX() - boxPath.get(i - 1).getX();
            double direction = Math.atan2(y, x);
            if (direction != currentDirection) {
                lines.add(new DirectionalLine(currentDirection, startPosition, boxPath.get(i - 1)));
                startPosition = boxPath.get(i - 1);
                currentDirection = direction;
            }
        }
        lines.add(new DirectionalLine(currentDirection, startPosition, boxPath.get(boxPath.size() - 1)));

        return lines;
    }

    private RobotConfig robotConfigFromBoxPos(Point2D boxPos, double movementDirection, double boxWidth) {
        double x = boxPos.getX() - (Math.cos(movementDirection) * ((boxWidth / 2) + 0.00005));
        double y = boxPos.getY() - (Math.sin(movementDirection) * ((boxWidth / 2) + 0.00005));
        double direction = movementDirection - (Math.PI / 2);
        return new RobotConfig(new double[] { Util.roundToStepSize(x, 0.000001), Util.roundToStepSize(y, 0.000001) },
                direction);
    }

    private Point2D boxPosFromRobotConfig(RobotConfig config, double movementDirection, double boxWidth) {
        double x = config.getPos().getX() + (Math.cos(movementDirection) * ((boxWidth / 2) + 0.00005));
        double y = config.getPos().getY() + (Math.sin(movementDirection) * ((boxWidth / 2) + 0.00005));
        return new Point2D.Double(Util.roundToStepSize(x, 0.000001), Util.roundToStepSize(y, 0.000001));
    }

    private void extendBoxPaths(List<List<Point2D>> boxPaths) {
        extendBoxPaths(boxPaths, -1, null, 0, 0);
    };

    private void extendBoxPaths(List<List<Point2D>> boxPaths, int indexToMove, RobotConfig configToMatch,
            double movementDirection, double boxWidth) {
        boxPaths.add(new ArrayList<>());
        int stepnr = boxPaths.size() - 1;
        for (int i = 0; i < ps.getMovingBoxes().size() + ps.getMovingObstacles().size(); i++) {
            if (i == indexToMove) {
                boxPaths.get(stepnr).add(boxPosFromRobotConfig(configToMatch, movementDirection, boxWidth));
            } else {
                boxPaths.get(stepnr).add(boxPaths.get(stepnr - 1).get(i));
            }
        }
    }

    public void outputPath(String outputFile) throws IOException {
        FileWriter fw = new FileWriter(outputFile);
        PrintWriter pw = new PrintWriter(fw);
        pw.println(robotPath.size());
        System.out.println(robotPath.size() + " | " + movingBoxPaths.size());
        for (int i = 0; i < robotPath.size(); i++) {
            double x = robotPath.get(i).getPos().getX();
            double y = robotPath.get(i).getPos().getY();
            double angle = robotPath.get(i).getOrientation();
            String obstacleString = "";
            for (int j = 0; j < ps.getMovingBoxes().size() + ps.getMovingObstacles().size(); j++) {
                obstacleString += " " + movingBoxPaths.get(i).get(j).getX() + " " + movingBoxPaths.get(i).get(j).getY();
            }

            pw.println(x + " " + y + " " + angle + obstacleString);
        }
        pw.close();
    }

}

class DirectionalLine {
    public Double direction;
    public Point2D startPosition;
    public Point2D endPosition;

    public DirectionalLine(Double direction, Point2D startPosition, Point2D endPosition) {
        this.direction = direction;
        this.startPosition = startPosition;
        this.endPosition = endPosition;
    }
}
