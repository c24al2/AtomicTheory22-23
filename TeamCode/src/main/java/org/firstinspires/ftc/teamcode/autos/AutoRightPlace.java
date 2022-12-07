package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.vision.ParkingPosition;


@Autonomous(name="Barcode Scan Right")
public class AutoRightPlace extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware vampire = new RobotHardware(hardwareMap, telemetry);
        setCurrentStep("Waiting for Start");
        waitForStart();

        sleep(4000);
        ParkingPosition parkingPosition = vampire.getParkingPosition();
        setCurrentStep("Going to place");


        vampire.driveRightSideMillimeters(-590,6000,0.7);
        sleep(300);
        vampire.driveFrontByMillimeters(610,3000,0.7);
        sleep(300);
        vampire.driveRightSideMillimeters(-260,4000,0.3);
        sleep(300);

        if (parkingPosition == ParkingPosition.ZONE1) {
            setCurrentStep("Going to Zone 1");
            vampire.driveRightSideMillimeters(305,5000,0.7);
            sleep(300);
        } else if (parkingPosition == ParkingPosition.ZONE2) {
            setCurrentStep("Going to Zone 2");
            vampire.driveRightSideMillimeters(915,5000,0.7);
            sleep(100);
        } else if (parkingPosition == ParkingPosition.ZONE3) {
            setCurrentStep("Going to Zone 3");
            vampire.driveRightSideMillimeters(1525,2000, .7);
            sleep(1000);
        } else if (parkingPosition == ParkingPosition.UNKNOWN) {
            setCurrentStep("Failed to identify zone :(");
            vampire.driveRightSide(1525, 2000, .4);
        }
    }

    void setCurrentStep(String currentStep) {
        telemetry.addData("Current Step: ", currentStep);
        telemetry.update();
    }
}
