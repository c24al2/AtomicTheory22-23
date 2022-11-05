package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "AutoBlueV1")
public class AutoBlueV1 extends LinearOpMode {
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
        vampire.driveRightSide(-2000,2000,0.5);
        vampire.lift(3000,3000,0.5);
    };



    }

