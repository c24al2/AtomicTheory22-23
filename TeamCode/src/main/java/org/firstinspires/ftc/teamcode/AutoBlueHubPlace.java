package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Barcode Scan and Place Blue Hub")
public class AutoBlueHubPlace extends LinearOpMode {
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
        sleep (4000);
        zonePark = vampire.getParkingPlace();
        currentStep = "goingToPlace";
        telemetry.addData("Current Task: ",currentStep);
        telemetry.update();
        vampire.driveRightSide(-1600,6000,0.7);
        sleep(300);
        vampire.driveFront(1950,3000,0.7);
        sleep(300);
        vampire.driveRightSide(-800,4000,0.3);
        sleep(300);
        vampire.lift(3000,3000,0.7);
        vampire.driveFront(300,1500,0.3);
        vampire.lift(-500,2000,0.7);
        vampire.lift(100,1000,0.3);
        vampire.driveFront(-300,3000,0.7);



        if (zonePark == 1){
            currentStep = "zone1";
            telemetry.addData("Current Task: ",currentStep);
            telemetry.update();
            vampire.driveRightSide(-800,5000,0.7);
            sleep(300);
        }
        else if (zonePark == 2){
            currentStep = "zone2";
            telemetry.addData("Current Task: ",currentStep);
            telemetry.update();
            vampire.driveRightSide(-1800,5000,0.7);
            sleep(100);
        }
        else if (zonePark == 3){
            currentStep = "zone3";
            telemetry.addData("Current Task: ",currentStep);
            telemetry.update();
            vampire.driveRightSide(-2700,2000, .7);
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
