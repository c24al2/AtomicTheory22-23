package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous (name = "Vampire Drive Back")
public class DriveBackAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware vampire = new RobotHardware(hardwareMap, telemetry);
        setCurrentStep("Waiting for Start");
        waitForStart();

        if (opModeIsActive()) {
            vampire.stopDrive();
            sleep(5000);
        }
    }

    void setCurrentStep(String currentStep) {
        telemetry.addData("Current Step: ", currentStep);
        telemetry.update();
    }
}



