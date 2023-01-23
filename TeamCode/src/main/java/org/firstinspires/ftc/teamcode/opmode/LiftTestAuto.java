package org.firstinspires.ftc.teamcode.opmode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.IntakeandLiftPID;
import org.firstinspires.ftc.teamcode.drive.SampleOmniDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@Autonomous
public class LiftTestAuto extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        SampleOmniDrive drive = new SampleOmniDrive(hardwareMap);
        IntakeandLiftPID liftandServo = new IntakeandLiftPID(hardwareMap);

//        liftandServo.intakeFullStep(liftandServo.inchesToTicks(DriveConstants.GROUNDJUNCTION));
//        liftandServo.intakeFullStep(liftandServo.inchesToTicks(DriveConstants.LOWJUNCTION));
//        liftandServo.intakeFullStep(liftandServo.inchesToTicks(DriveConstants.MEDIUMJUNCTION));
//        liftandServo.intakeFullStep(liftandServo.inchesToTicks(DriveConstants.HIGHJUNCTION));
        sleep(4000);
    }



}
