package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="BarcodeScanAutoBlue")
public class BarcodeScanAutoBlue extends LinearOpMode {
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
            vampire.driveRightSide(-1600,10000,0.7);
            sleep(300);
            vampire.driveFront(1900,3000,0.7);
            sleep(1000);
        }
        else if (zonePark == 2){
            currentStep = "zone2";
            telemetry.addData("Current Task: ",currentStep);
            telemetry.update();
            vampire.driveFront(1900,3000,0.7);
            sleep(100);
        }
        else if (zonePark == 3){
            currentStep = "zone3";
            telemetry.addData("Current Task: ",currentStep);
            telemetry.update();
            vampire.driveRightSide(1700,2000, .7);
            sleep(300);
            vampire.driveFront(1900,3000,0.7);
            sleep(1000);
        }
        else if (zonePark == 0){
            currentStep = "failed :(";
            vampire.driveRightSide(-2700, 2000, .4);
            telemetry.addData("Current Task: ",currentStep);
            telemetry.update();
        }
    }

}
