package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous (name = "AutoBlueV1")
public class AutoBlueV1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware vampire = new RobotHardware(hardwareMap, telemetry);
        setCurrentStep("Waiting for Start");
        waitForStart();

        vampire.driveRightSide(-2000,2000,0.5);
        vampire.lift(3000,3000,0.5);
    }

    void setCurrentStep(String currentStep) {
        telemetry.addData("Current Step: ", currentStep);
        telemetry.update();
    }
}

