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
    List<RobotConfig> robotPath;
    List<List<Point2D>> movingBoxPaths; // row = stepnr | column = boxIndex

    public Planner(ProblemSpec ps) {
        this.ps = ps;
        rrt = new RRT();

    }

    public void plan() throws IOException {
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
        }
        for (StaticObstacle box : ps.getStaticObstacles()) {
            obstacles.add(box.getRect());
        }

        List<List<Point2D>> boxPaths = dummy1();
        List<Integer> indexList = dummy3();
        for (int i = 0; i < boxPaths.size(); i++) {
            List<Point2D> path = boxPaths.get(i);
            int boxIndex = indexList.get(i);

            obstacles.remove(Util.pointToRect(path.get(0), ps.getRobotWidth()));

            Point2D failPoint = calcPaths(boxIndex, path, obstacles);
            while (failPoint != null) {
                System.out.println("failed at: " + failPoint);
                List<Point2D> newPath = dummy2(failPoint, boxIndex);
                if (newPath == null) {
                    System.err.println("no solution");
                    return;
                }
                failPoint = calcPaths(boxIndex, newPath, obstacles);
            }
            obstacles.add(Util.pointToRect(path.get(path.size()), ps.getRobotWidth()));
        }

        Point2D start = ps.getMovingBoxes().get(0).getPos();
        start.setLocation(start.getX() + (ps.getRobotWidth() / 2), start.getY() + (ps.getRobotWidth() / 2));
        Point2D goal = ps.getMovingBoxEndPositions().get(0);
        goal.setLocation(goal.getX() + (ps.getRobotWidth() / 2), goal.getY() + (ps.getRobotWidth() / 2));

        Astar agent = new Astar();
        double gridWidth = ps.getRobotWidth();
        List<Point2D> path = ((List<GridNode>) agent.search(start, goal, gridWidth)).stream().map(x -> x.pos)
                .collect(Collectors.toList());
        // for (int i = 0; i < path.size(); i++) {
        // System.out.println(path.get(i));
        // }
        obstacles.remove(Util.pointToRect(start, ps.getRobotWidth()));
        Point2D failPoint = calcPaths(0, path, obstacles);
        if (failPoint != null) {

            System.out.println("failed at: " + failPoint);
        }
        obstacles.add(Util.pointToRect(goal, ps.getRobotWidth()));

        movingBoxPaths.remove(0);

        outputPath();
    }

    private List<List<Point2D>> dummy1() {
        return null;
    }

    private List<Point2D> dummy2(Point2D point, int boxIndex) {
        return null;
    }

    private List<Integer> dummy3() {
        return null;
    }

    private Point2D calcPaths(int boxIndex, List<Point2D> boxPath, List<Rectangle2D> staticObstacles) {
        List<DirectionalLine> lines = splitByDirection(boxPath);
        RobotConfig currentConfig = ps.getInitialRobotConfig();
        double robotWidth = ps.getRobotWidth();
        List<RobotConfig> tempRobotPath = new ArrayList<>();
        List<List<Point2D>> tempMovingBoxPaths = new ArrayList<>();
        tempMovingBoxPaths.add(new ArrayList<>());
        for (int i = 0; i < ps.getMovingBoxes().size(); i++) {
            tempMovingBoxPaths.get(0).add(movingBoxPaths.get(movingBoxPaths.size() - 1).get(i));
        }
        for (int i = 0; i < lines.size(); i++) {
            DirectionalLine line = lines.get(i);
            RobotConfig nextConfig = robotConfigFromBoxPos(line.startPosition, line.direction);
            List<Rectangle2D> obstacles = new ArrayList<>();
            obstacles.addAll(staticObstacles);
            obstacles.add(Util.pointToRect(line.startPosition, robotWidth));
            List<RobotConfig> pathToStart = rrt.calculatePath(currentConfig, nextConfig, robotWidth, obstacles);
            if (pathToStart == null) {
                return line.startPosition;
            }
            tempRobotPath.addAll(pathToStart);

            while (tempMovingBoxPaths.size() < tempRobotPath.size()) {
                extendBoxPaths(tempMovingBoxPaths);
            }
            currentConfig = nextConfig;
            nextConfig = robotConfigFromBoxPos(line.endPosition, line.direction);
            addDirectionalLineToPaths(tempRobotPath, tempMovingBoxPaths, boxIndex, line.direction, currentConfig,
                    nextConfig);
            currentConfig = nextConfig;
        }
        robotPath.addAll(tempRobotPath);

        movingBoxPaths.addAll(tempMovingBoxPaths);

        return null;

    }

    private void addDirectionalLineToPaths(List<RobotConfig> tempRobotPath, List<List<Point2D>> tempMovingBoxPaths,
            int boxIndex, double movementDirection, RobotConfig startConfig, RobotConfig goalConfig) {
        double distance = startConfig.getPos().distance(goalConfig.getPos());
        int steps = (int) Math.floor(distance / 0.001);
        RobotConfig currentConfig = startConfig;
        tempRobotPath.add(startConfig);
        extendBoxPaths(tempMovingBoxPaths);
        for (int i = 0; i < steps; i++) {
            currentConfig = new RobotConfig(Util.translateOneStep(currentConfig.getPos(), movementDirection),
                    currentConfig.getOrientation());
            tempRobotPath.add(currentConfig);
            extendBoxPaths(tempMovingBoxPaths, boxIndex, currentConfig, movementDirection);
        }
        if (!currentConfig.getPos().equals(goalConfig.getPos())) {
            tempRobotPath.add(goalConfig);
            extendBoxPaths(tempMovingBoxPaths, boxIndex, goalConfig, movementDirection);
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

    private RobotConfig robotConfigFromBoxPos(Point2D boxPos, double movementDirection) {
        double x = boxPos.getX() - (Math.cos(movementDirection) * (ps.getRobotWidth() / 2));
        double y = boxPos.getY() - (Math.sin(movementDirection) * (ps.getRobotWidth() / 2));
        double direction = movementDirection - (Math.PI / 2);
        return new RobotConfig(new double[] { x, y }, direction);
    }

    private Point2D boxPosFromRobotConfig(RobotConfig config, double movementDirection) {
        double x = config.getPos().getX() + (Math.cos(movementDirection) * (ps.getRobotWidth() / 2));
        double y = config.getPos().getY() + (Math.sin(movementDirection) * (ps.getRobotWidth() / 2));
        return new Point2D.Double(x, y);
    }

    private void extendBoxPaths(List<List<Point2D>> boxPaths) {
        extendBoxPaths(boxPaths, -1, null, 0);
    };

    private void extendBoxPaths(List<List<Point2D>> boxPaths, int indexToMove, RobotConfig configToMatch,
            double movementDirection) {
        boxPaths.add(new ArrayList<>());
        int stepnr = boxPaths.size() - 1;
        for (int i = 0; i < ps.getMovingBoxes().size(); i++) {
            if (i == indexToMove) {
                boxPaths.get(stepnr).add(boxPosFromRobotConfig(configToMatch, movementDirection));
            } else {
                boxPaths.get(stepnr).add(boxPaths.get(stepnr - 1).get(i));
            }
        }
    }

    public void outputPath() throws IOException {
        FileWriter fw = new FileWriter("output.txt");
        PrintWriter pw = new PrintWriter(fw);
        pw.println(robotPath.size());
        System.out.println(robotPath.size() + " | " + movingBoxPaths.size());
        for (int i = 0; i < robotPath.size(); i++) {
            double x = robotPath.get(i).getPos().getX();
            double y = robotPath.get(i).getPos().getY();
            double angle = robotPath.get(i).getOrientation();
            String obstacleString = "";
            for (int j = 0; j < ps.getMovingBoxes().size(); j++) {
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