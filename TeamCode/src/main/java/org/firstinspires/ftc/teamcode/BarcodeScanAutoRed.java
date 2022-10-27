package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="BarcodeScanAutoRed")
public class BarcodeScanAutoRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        int zonePark;
        RobotHardware vampire = new RobotHardware(hardwareMap, telemetry);
        waitForStart();
        ElapsedTime runtime = new ElapsedTime();
        String currentStep = "Waiting for Start";
        Thread telemetryHandler = new Thread(){
            @Override
            public void run() {
                while (opModeIsActive()) {
                    telemetry.addData("Runtime(s): ", runtime.seconds());
                    telemetry.update();
                }
            }
        };
        sleep (5000);
        zonePark = vampire.getParkingPlace();
        if (zonePark == 1){
            currentStep = "zone1";
            telemetry.addData("Current Task: ",currentStep);
            telemetry.update();
            vampire.driveLeftSide(500);
            sleep(300);
            vampire.driveFront(500);
            sleep(100);
        }
        else if (zonePark == 2){
            currentStep = "zone2";
            telemetry.addData("Current Task: ",currentStep);
            telemetry.update();
            vampire.driveFront(500);
            sleep(100);
        }
        else if (zonePark == 3){
            currentStep = "zone1";
            telemetry.addData("Current Task: ",currentStep);
            telemetry.update();
            vampire.driveRightSide(500);
            sleep(300);
            vampire.driveFront(500);
            sleep(100);
        }
        else if (zonePark == 0){
            currentStep = "failed :(";
            vampire.driveRightSide(700);
            telemetry.addData("Current Task: ",currentStep);
            telemetry.update();
        }
    }

}
