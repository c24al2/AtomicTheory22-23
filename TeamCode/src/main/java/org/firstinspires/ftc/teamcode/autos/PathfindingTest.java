package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.Constants.FIELD_OBSTACLES;
import static org.firstinspires.ftc.teamcode.Constants.FT_TO_CM;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.pathfinding.Pathfinder;
import org.firstinspires.ftc.teamcode.pathfinding.Node;
import org.firstinspires.ftc.teamcode.pathfinding.Obstacle;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Pathfinding Test")
public class PathfindingTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        setCurrentStep("Waiting for Start");
        waitForStart();
        setCurrentStep("Creating pathfinder");

        List<Obstacle> obstacles = Arrays.asList(FIELD_OBSTACLES);
        Pathfinder pathfinder = new Pathfinder(obstacles);

        setCurrentStep("Finding best path");
        Node start = Node.createFromCM(0, 0);
        Node goal = Node.createFromCM(11.25*FT_TO_CM, 11.25*FT_TO_CM);
        List<Node> path = pathfinder.findPath(start, goal);
        telemetry.addData("Path Length: ", path.size());
        telemetry.update();

        setCurrentStep("Fitting function to path");
        double[] coefficients = pathfinder.fitPolynomialToPath(path, 4);

        StringBuilder polynomial = new StringBuilder();
        for (int i = coefficients.length-1; i >= 0; i--) {
            polynomial.append(String.format("%.9fx^%d", coefficients[i], i));
            if (i > 0) {
                polynomial.append(" + ");
            }
        }
        telemetry.addData("Polynomial: ", polynomial.toString());
        telemetry.update();
    }

    void setCurrentStep(String currentStep) {
        telemetry.addData("Current Step: ", currentStep);
        telemetry.update();
    }
}