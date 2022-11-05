package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "Vampire Drive Back")
public class driveBackAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware vampire = new RobotHardware(hardwareMap, telemetry);
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
        if (opModeIsActive()) {
            vampire.stopDrive();
            sleep(5000);
        }

    };



}



