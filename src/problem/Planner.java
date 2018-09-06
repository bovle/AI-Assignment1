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
    List<List<Box>> movingBoxPaths;

    public Planner(ProblemSpec ps) {
        this.ps = ps;
        rrt = new RRT();
        robotPath = new ArrayList<>();
        movingBoxPaths = new ArrayList<>();
    }

    public void plan() throws IOException {
        List<Rectangle2D> obstacles = new ArrayList<>();
        for (Box box : ps.getMovingBoxes()) {
            System.out.println(box.getPos());
            obstacles.add(box.getRect());
            List<Box> boxPath = new ArrayList<>();
            boxPath.add(box);
            movingBoxPaths.add(boxPath);

        }
        for (Box box : ps.getMovingObstacles()) {
            obstacles.add(box.getRect());
        }
        for (StaticObstacle box : ps.getStaticObstacles()) {
            obstacles.add(box.getRect());
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
        calcPaths(path, obstacles);
        obstacles.add(Util.pointToRect(goal, ps.getRobotWidth()));

        outputPath();
    }

    private boolean calcPaths(List<Point2D> boxPath, List<Rectangle2D> staticObstacles) {
        List<DirectionalLine> lines = splitByDirection(boxPath);
        RobotConfig currentConfig = ps.getInitialRobotConfig();
        double robotWidth = ps.getRobotWidth();
        for (int i = 0; i < lines.size(); i++) {
            DirectionalLine line = lines.get(i);
            RobotConfig nextConfig = robotConfigFromBoxPos(line.startPosition, line.direction);
            List<Rectangle2D> obstacles = new ArrayList<>();
            obstacles.addAll(staticObstacles);
            obstacles.add(Util.pointToRect(line.startPosition, robotWidth));
            List<RobotConfig> pathToStart = rrt.calculatePath(currentConfig, nextConfig, robotWidth, obstacles);
            if (pathToStart == null) {
                return false;
            }
            robotPath.addAll(pathToStart);

            while (movingBoxPaths.get(0).size() < robotPath.size()) {
                extendBoxPaths();
            }
            currentConfig = nextConfig;
            nextConfig = robotConfigFromBoxPos(line.endPosition, line.direction);
            addDirectionalLineToPaths(line.direction, currentConfig, nextConfig);
            currentConfig = nextConfig;
        }
        return true;

    }

    private void addDirectionalLineToPaths(double movementDirection, RobotConfig startConfig, RobotConfig goalConfig) {
        double distance = startConfig.getPos().distance(goalConfig.getPos());
        int steps = (int) Math.floor(distance / 0.001);
        RobotConfig currentConfig = startConfig;
        robotPath.add(startConfig);
        extendBoxPaths();
        for (int i = 0; i < steps; i++) {
            currentConfig = new RobotConfig(Util.translateOneStep(currentConfig.getPos(), movementDirection),
                    currentConfig.getOrientation());
            robotPath.add(currentConfig);
            extendBoxPaths(0, currentConfig, movementDirection);
        }
        if (!currentConfig.getPos().equals(goalConfig.getPos())) {
            robotPath.add(goalConfig);
            extendBoxPaths(0, goalConfig, movementDirection);
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
        return new Point2D.Double(x - (ps.getRobotWidth() / 2), y - (ps.getRobotWidth() / 2));
    }

    private void extendBoxPaths() {
        extendBoxPaths(-1, null, 0);
    };

    private void extendBoxPaths(int indexToMove, RobotConfig configToMatch, double movementDirection) {
        for (int i = 0; i < movingBoxPaths.size(); i++) {
            List<Box> boxPath = movingBoxPaths.get(i);
            if (i == indexToMove) {
                boxPath.add(new MovingBox(boxPosFromRobotConfig(configToMatch, movementDirection), ps.getRobotWidth()));
            } else {
                boxPath.add(boxPath.get(boxPath.size() - 1));
            }
        }
    }

    public void outputPath() throws IOException {
        FileWriter fw = new FileWriter("output.txt");
        PrintWriter pw = new PrintWriter(fw);
        pw.println(robotPath.size());
        System.out.println(robotPath.size() + " | " + movingBoxPaths.get(0).size());
        for (int i = 0; i < robotPath.size(); i++) {
            double x = robotPath.get(i).getPos().getX();
            double y = robotPath.get(i).getPos().getY();
            double angle = robotPath.get(i).getOrientation();
            String obstacleString = "";
            for (int j = 0; j < movingBoxPaths.size(); j++) {
                obstacleString += " " + movingBoxPaths.get(j).get(i).getRect().getCenterX() + " "
                        + movingBoxPaths.get(j).get(i).getRect().getCenterY();
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