package org.firstinspires.ftc.teamcode.opmode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.IntakeandLiftPID;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleOmniDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.CameraController;
import org.firstinspires.ftc.teamcode.vision.ParkingPositionPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@Autonomous
public class LiftTestAuto extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        SampleOmniDrive drive = new SampleOmniDrive(hardwareMap);
        IntakeandLiftPID liftandServo = new IntakeandLiftPID(hardwareMap);

        liftandServo.intakeFullStep(DriveConstants.GROUNDJUNCTION);
        liftandServo.intakeFullStep(DriveConstants.LOWJUNCTION);
        liftandServo.intakeFullStep(DriveConstants.MEDIUMJUNCTION);
        liftandServo.intakeFullStep(DriveConstants.HIGHJUNCTION);
        sleep(4000);
    }



}
