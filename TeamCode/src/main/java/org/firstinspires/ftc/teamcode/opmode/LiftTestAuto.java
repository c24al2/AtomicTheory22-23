package org.firstinspires.ftc.teamcode.opmode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleOmniDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@Autonomous
public class LiftTestAuto extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        SampleOmniDrive drive = new SampleOmniDrive(hardwareMap);
        Lift liftandServo = new Lift(hardwareMap);

        liftandServo.intakeFullStep(DriveConstants.GROUNDJUNCTION);
        liftandServo.intakeFullStep(DriveConstants.LOWJUNCTION);
        liftandServo.intakeFullStep(DriveConstants.MEDIUMJUNCTION);
        liftandServo.intakeFullStep(DriveConstants.HIGHJUNCTION);
        sleep(4000);
    }



}
