package problem;

import java.awt.geom.Rectangle2D;
import java.awt.geom.Point2D;
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
            ps.loadProblem(args[0]);
            Planner planner = new Planner(ps);
            planner.plan();

        } catch (IOException e) {
            System.out.println("IO Exception occured");
        }

    }

}
