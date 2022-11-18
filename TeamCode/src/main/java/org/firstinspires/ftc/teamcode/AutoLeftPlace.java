package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Barcode Scan Left")
public class AutoLeftPlace extends LinearOpMode {
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
        vampire.lift(100,500,0.7);
        vampire.driveRightSideMillimeters(610,6000,0.7);
        sleep(300);
        vampire.driveFrontByMillimeters(610,3000,0.7);
        sleep(300);
        vampire.driveRightSideMillimeters(305,4000,0.3);
        sleep(300);
        vampire.lift(3000,3000,0.7);
        vampire.driveFrontByMillimeters(63,1000,0.3);
        vampire.lift(-500,2000,0.7);
        vampire.lift(100,1000,0.3);
        vampire.driveFrontByMillimeters(-63,3000,0.7);



        if (zonePark == 1){
            currentStep = "zone1";
            telemetry.addData("Current Task: ",currentStep);
            telemetry.update();
            vampire.driveRightSideMillimeters(-305,5000,0.7);
            sleep(300);
        }
        else if (zonePark == 2){
            currentStep = "zone2";
            telemetry.addData("Current Task: ",currentStep);
            telemetry.update();
            vampire.driveRightSideMillimeters(-915,5000,0.7);
            sleep(100);
        }
        else if (zonePark == 3){
            currentStep = "zone3";
            telemetry.addData("Current Task: ",currentStep);
            telemetry.update();
            vampire.driveRightSideMillimeters(-1525,2000, .7);
            sleep(1000);
        }
        else if (zonePark == 0){
            currentStep = "failed :(";
            vampire.driveRightSide(-1525, 2000, .4);
            telemetry.addData("Current Task: ",currentStep);
            telemetry.update();
        }
    }

}
