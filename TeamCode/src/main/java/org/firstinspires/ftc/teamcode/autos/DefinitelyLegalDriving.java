package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="Definitely Legal Driving")
public class DefinitelyLegalDriving extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware vampire = new RobotHardware(hardwareMap, telemetry);
        setCurrentStep("Waiting for Start");
        waitForStart();

        sleep(200);
    }

    void setCurrentStep(String currentStep) {
        telemetry.addData("Current Step: ", currentStep);
        telemetry.update();
    }
}


