package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.Constants.FIELD_OBSTACLES;
import static org.firstinspires.ftc.teamcode.Constants.FT_TO_CM;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.pathfinding.Pathfinder;
import org.firstinspires.ftc.teamcode.pathfinding.Node;
import org.firstinspires.ftc.teamcode.pathfinding.Obstacle;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Pathfinding Practical Test")
public class PathfindingPracticalTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware vampire = new RobotHardware(hardwareMap, telemetry);
        setCurrentStep("Waiting for Start");
        waitForStart();
        setCurrentStep("Creating pathfinder");

        List<Obstacle> obstacles = Arrays.asList(FIELD_OBSTACLES);
        Pathfinder pathfinder = new Pathfinder(obstacles);

        setCurrentStep("Finding best path");
        Node start = Node.createFromCM(0, 0);
        Node goal = Node.createFromCM(5*FT_TO_CM, 2*FT_TO_CM);
        List<Node> path = pathfinder.findPath(start, goal);
        telemetry.addData("Path Length: ", path.size());
        telemetry.update();
        telemetry.addData("Current Step", "running along path");
        vampire.PIDAStarRevamp(path);
    }

    void setCurrentStep(String currentStep) {
        telemetry.addData("Current Step: ", currentStep);
        telemetry.update();
    }
}
