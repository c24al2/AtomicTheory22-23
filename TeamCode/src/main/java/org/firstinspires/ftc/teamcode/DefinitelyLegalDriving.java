package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "Doug")
public class DefinitelyLegalDriving extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware doug = new RobotHardware(hardwareMap, telemetry);
        waitForStart();
        ElapsedTime runtime = new ElapsedTime();
        String currentStep = "Waiting for Start";
        Thread telemetryHandler = new Thread(){
            @Override
            public void run() {
                while (opModeIsActive()) {
                    telemetry.addData("Current Task: ", currentStep);
                    telemetry.addData("Runtime(s): ", runtime.seconds());
                    telemetry.update();
                }

            }
        };
        doug.driveFront(2500);
        sleep(10);
        doug.driveLeftSide(2500);
        sleep(10);
        doug.driveBack(2500);
        sleep(10);
        doug.driveRightSide(1000);
        sleep(10);
        doug.driveFront(1000);

    };



}

