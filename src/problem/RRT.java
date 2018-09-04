package problem;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import javax.lang.model.util.ElementScanner6;

public class RRT {
    public double robotWidth;
    public double angleStepSize;
    public List<Rectangle2D> obstacles;
    public List<TreeNode> startTree;
    public List<TreeNode> goalTree;

    public List<RobotConfig> calculatePath(RobotConfig startConfig, RobotConfig goalConfig, double robotWidth,
            List<Rectangle2D> obstacles) {

        this.robotWidth = robotWidth;
        this.angleStepSize = Math.asin(0.0005 / (robotWidth / 2)) * 2;
        this.obstacles = obstacles;
        this.startTree = new ArrayList<>();
        this.goalTree = new ArrayList<>();

        if (!collisionFree(startConfig)) {
            System.err.println("start config in collision");
            return null;
        }
        if (!collisionFree(goalConfig)) {
            System.err.println("goal config in collision");
            return null;
        }

        List<RobotConfig> path = new ArrayList<>();

        startTree.add(new TreeNode(startConfig, null, null));
        goalTree.add(new TreeNode(goalConfig, null, null));

        if (doubelRRT(100000, 20)) {

            List<RobotConfig> startToMiddle = treeToPath(startTree.get(startTree.size() - 1), startTree.get(0));
            Collections.reverse(startToMiddle);
            List<RobotConfig> middleToGoal = treeToPath(goalTree.get(goalTree.size() - 1), goalTree.get(0));
            path.addAll(startToMiddle);
            path.addAll(middleToGoal.subList(1, middleToGoal.size() - 1));
            System.out.println("success");
        } else {
            System.out.println("failure");
        }
        return path;
    }

    private List<RobotConfig> treeToPath(TreeNode leaf, TreeNode root) {
        List<RobotConfig> path = new ArrayList<>();
        path.add(leaf.data);
        TreeNode current = leaf;
        while (!current.data.equals(root.data)) {
            path.addAll(current.pathToParent);
            current = current.parent;
        }
        return path;
    }

    private boolean doubelRRT(int steps, int stepSize) {
        Random r = new Random();
        for (int i = 0; i < steps; i++) {
            RobotConfig randConfig = new RobotConfig(new double[] { r.nextDouble(), r.nextDouble() },
                    (r.nextDouble() * (Math.PI * 2)) - Math.PI);
            if (i % 2 == 0) {
                TreeNode newNode1 = extendRRT(startTree, randConfig, stepSize);
                if (newNode1 != null) {
                    if (connectRRT(goalTree, newNode1.data, stepSize))
                        return true;
                }
            } else {
                TreeNode newNode1 = extendRRT(goalTree, randConfig, stepSize);
                if (newNode1 != null) {
                    if (connectRRT(startTree, newNode1.data, stepSize))
                        return true;
                }
            }
        }
        return false;
    }

    private boolean connectRRT(List<TreeNode> tree, RobotConfig targetConfig, int stepSize) {
        TreeNode stepTreeNode;
        int counter = 0;
        while (counter < 1000) {
            counter++;
            stepTreeNode = extendRRT(tree, targetConfig, stepSize);

            if (stepTreeNode == null)
                return false;
            if (stepTreeNode.data.equals(targetConfig))
                return true;
        }
        return false;
    }

    private TreeNode extendRRT(List<TreeNode> tree, RobotConfig randConfig, int stepSize) {
        TreeNode closestNode = tree.get(0);
        double shortestStepDist = stepDistance(randConfig, closestNode.data);
        for (int i = 1; i < tree.size(); i++) {
            double currentStepDist = stepDistance(randConfig, tree.get(i).data);
            if (currentStepDist < shortestStepDist) {
                closestNode = tree.get(i);
                shortestStepDist = currentStepDist;
            }
        }
        List<RobotConfig> directPath = directPath(closestNode.data, randConfig);
        RobotConfig newConfig;
        List<RobotConfig> subPath;
        if (directPath.size() <= stepSize) {
            newConfig = directPath.get(directPath.size() - 1);
            subPath = directPath;
        } else {
            newConfig = directPath.get(stepSize - 1);
            subPath = directPath.subList(0, stepSize);
        }
        if (collisionFreeLine(subPath)) {
            Collections.reverse(subPath);
            TreeNode newNode = new TreeNode(newConfig, closestNode, subPath);
            tree.add(newNode);
            return newNode;
        }
        return null;

    }

    public boolean collisionFreeLine(List<RobotConfig> path) {
        if (!collisionFree(path.get(0)) || !collisionFree(path.get(path.size() - 1))) {
            return false;
        }
        if (path.size() <= 2)
            return true;
        int distToMidPoint = (int) (path.size() / 2.0);
        double aDistToNearestForbidden = distToNearestForbidden(path.get(0)) / 0.001;
        double bDistToNearestForbidden = distToNearestForbidden(path.get(path.size() - 1)) / 0.001;
        if (distToMidPoint <= aDistToNearestForbidden && (path.size() - distToMidPoint) <= bDistToNearestForbidden) {
            return true;
        } else
            return collisionFreeLine(path.subList(0, distToMidPoint + 1))
                    && collisionFreeLine(path.subList(distToMidPoint, path.size()));

    }

    public boolean collisionFree(RobotConfig config) {
        double x1 = config.getX1(robotWidth);
        double x2 = config.getX2(robotWidth);
        double y1 = config.getY1(robotWidth);
        double y2 = config.getY2(robotWidth);
        Line2D robotLine = new Line2D.Double(x1, y1, x2, y2);
        Rectangle2D border = new Rectangle2D.Double(0, 0, 1, 1);
        if (!border.contains(robotLine.getP1()) || !border.contains(robotLine.getP2()))
            return false;

        for (int i = 0; i < obstacles.size(); i++) {
            if (grow(obstacles.get(i), -0.0001).intersectsLine(robotLine))
                return false;
        }

        return true;
    }

    public Rectangle2D grow(Rectangle2D rect, double delta) {
        return new Rectangle2D.Double(rect.getX() - delta, rect.getY() - delta, rect.getWidth() + 2 * delta,
                rect.getHeight() + 2 * delta);
    }

    public double distToNearestForbidden(RobotConfig config) {
        double shortestDist = Double.MAX_VALUE;
        Line2D configLine = new Line2D.Double(config.getX1(robotWidth), config.getY1(robotWidth),
                config.getX2(robotWidth), config.getY2(robotWidth));

        for (Rectangle2D rect : obstacles) {
            double dist = lineToRectDist(configLine, rect);
            if (dist < shortestDist)
                shortestDist = dist;
        }
        Rectangle2D border = new Rectangle2D.Double(0, 0, 1, 1);
        double dist = lineToRectDist(configLine, border);
        if (dist < shortestDist)
            shortestDist = dist;

        return shortestDist;
    }

    private double lineToRectDist(Line2D line, Rectangle2D rect) {
        double shortestDist = Double.MAX_VALUE;
        Point2D lineEndPoints[] = { line.getP1(), line.getP2() };
        Point2D rectEndPoints[] = { new Point2D.Double(rect.getMinX(), rect.getMinY()),
                new Point2D.Double(rect.getMinX(), rect.getMaxY()), new Point2D.Double(rect.getMaxX(), rect.getMaxY()),
                new Point2D.Double(rect.getMaxX(), rect.getMinY()) };

        Line2D[] rectLines = new Line2D.Double[4];
        for (int i = 0; i < 3; i++) {
            rectLines[i] = new Line2D.Double(rectEndPoints[i], rectEndPoints[i + 1]);
        }
        rectLines[3] = new Line2D.Double(rectEndPoints[3], rectEndPoints[0]);

        // line points dist to rect
        for (int i = 0; i < lineEndPoints.length; i++) {
            for (int j = 0; j < rectLines.length; j++) {
                double dist = rectLines[j].ptSegDist(lineEndPoints[i]);
                if (dist < shortestDist)
                    shortestDist = dist;
            }
        }

        // rect points dist to line
        for (int i = 0; i < rectEndPoints.length; i++) {
            double dist = line.ptSegDist(rectEndPoints[i]);
            if (dist < shortestDist)
                shortestDist = dist;
        }
        return shortestDist - 0.0001;
    }

    public List<RobotConfig> directPath(RobotConfig startConfig, RobotConfig goalConfig) {
        List<RobotConfig> path = new ArrayList<>();
        double translationDist = startConfig.getPos().distance(goalConfig.getPos());
        double xDif = goalConfig.getPos().getX() - startConfig.getPos().getX();
        double yDif = goalConfig.getPos().getY() - startConfig.getPos().getY();
        double translationAngle = Math.atan2(yDif, xDif);

        int translationSteps = (int) Math.floor(translationDist / 0.001);

        double rotationalDist = goalConfig.getOrientation() - startConfig.getOrientation();
        if (rotationalDist < -Math.PI)
            rotationalDist += Math.PI * 2;
        else if (rotationalDist > Math.PI)
            rotationalDist -= Math.PI * 2;
        int rotationalSteps = (int) Math.floor(Math.abs(rotationalDist / angleStepSize));

        int totalSteps = translationSteps + rotationalSteps;
        double translationPart = ((double) translationSteps) / totalSteps;
        double rotationPart = ((double) rotationalSteps) / totalSteps;
        double currentTranslationPart = translationPart;
        double currentRotationPart = rotationPart;
        RobotConfig currentConfig = startConfig;
        path.add(currentConfig);
        for (int i = 0; i < totalSteps; i++) {
            if (currentTranslationPart >= currentRotationPart) {
                currentConfig = translate(currentConfig, translationAngle);
                currentTranslationPart -= 1;
            } else {
                currentConfig = rotate(currentConfig, rotationalDist < 0);
                currentRotationPart -= 1;
            }
            currentTranslationPart += translationPart;
            currentRotationPart += rotationPart;
            path.add(currentConfig);
        }
        if (currentConfig.getPos().distance(goalConfig.getPos()) > 0.0001) {
            currentConfig = new RobotConfig(goalConfig.getPos(), currentConfig.getOrientation());
            path.add(currentConfig);
        }
        double angleError = Math.asin((0.0001 / 2) / (robotWidth / 2)) * 2;
        if (Math.abs(currentConfig.getOrientation() - goalConfig.getOrientation()) > angleError) {
            currentConfig = new RobotConfig(currentConfig.getPos(), goalConfig.getOrientation());
            path.add(currentConfig);
        }
        return path;
    }

    private RobotConfig translate(RobotConfig config, double angle) {
        double nextX = 0.001 * Math.cos(angle) + config.getPos().getX();
        double nextY = 0.001 * Math.sin(angle) + config.getPos().getY();
        return new RobotConfig(new double[] { nextX, nextY }, config.getOrientation());
    }

    private RobotConfig rotate(RobotConfig config, boolean isClockWise) {
        double angleStep = 0;
        if (isClockWise)
            angleStep = angleStepSize * -1;
        else
            angleStep = angleStepSize;
        double newAngle = config.getOrientation() + angleStep;
        if (newAngle > Math.PI)
            newAngle -= Math.PI * 2;
        else if (newAngle < -Math.PI)
            newAngle += Math.PI * 2;
        return new RobotConfig((Point2D) config.getPos().clone(), newAngle);
    }

    public double stepDistance(RobotConfig configA, RobotConfig configB) {
        double translationDist = configB.getPos().distance(configA.getPos());
        double translationSteps = translationDist / 0.001;
        double rotationalDist = configB.getOrientation() - configA.getOrientation();
        if (rotationalDist < -Math.PI)
            rotationalDist += Math.PI * 2;
        else if (rotationalDist > Math.PI)
            rotationalDist -= Math.PI * 2;
        double rotationalSteps = Math.abs(rotationalDist / angleStepSize);

        return translationSteps + rotationalSteps;
    }

}