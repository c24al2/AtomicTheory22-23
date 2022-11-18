package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.vision.ParkingPosition;

@Autonomous(name="BarcodeScanAutoRed")
public class BarcodeScanAutoRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware vampire = new RobotHardware(hardwareMap, telemetry);
        setCurrentStep("Waiting for Start");
        waitForStart();

        sleep(5000);
        ParkingPosition parkingPosition = vampire.getParkingPosition();
        if (parkingPosition == ParkingPosition.ZONE1){
            setCurrentStep("Going to Zone 1");
            vampire.driveRightSide(-1600,10000,0.7);
            sleep(300);
            vampire.driveFront(1900,3000,0.7);
            sleep(1000);
        } else if (parkingPosition == ParkingPosition.ZONE2){
            setCurrentStep("Going to Zone 2");
            vampire.driveFront(1900,3000,0.7);
            sleep(100);
        } else if (parkingPosition == ParkingPosition.ZONE3){
            setCurrentStep("Going to Zone 3");
            vampire.driveRightSide(1700,2000, .7);
            sleep(300);
            vampire.driveFront(1900,3000,0.7);
            sleep(1000);
        } else if (parkingPosition == ParkingPosition.UNKNOWN){
            setCurrentStep("Failed to identify zone :(");
            vampire.driveRightSide(-2700, 2000, .4);
        }
    }

    void setCurrentStep(String currentStep) {
        telemetry.addData("Current Step: ", currentStep);
        telemetry.update();
    }
}
