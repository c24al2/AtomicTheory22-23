package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleOmniDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.ParkingPositionPipeline;

@Config
@Autonomous
public class RightParkingAuto extends LinearOpMode {
    // These can be changed using FTC Dashboard!
    public static Pose2d START_POSE = new Pose2d(-62.8, -35, Math.toRadians(0));
//    public static Pose2d PLACE_PRELOADED_CONE_POSE = new Pose2d(-1, -17, Math.toRadians(-46));
//    public static Pose2d PICKUP_CONE_FROM_STACK_POSE = new Pose2d(-8, -64, Math.toRadians(-90));
//    public static Pose2d PLACE_STACK_CONE_POSE = new Pose2d(-1, -31, Math.toRadians(46));

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

//        CameraController cameraController = new CameraController(hardwareMap);
        ParkingPositionPipeline aprilTagPipeline = new ParkingPositionPipeline();
//        cameraController.setPipeline(aprilTagPipeline);

        telemetry.addData("Parking Position", aprilTagPipeline.parkingPosition);
        telemetry.update();

//        cameraController.stopStreaming(); // Reduce resource usage by not processing any more camera

        SampleOmniDrive drive = new SampleOmniDrive(hardwareMap);
//        IntakeandLiftPID liftandServo = new IntakeandLiftPID(hardwareMap);
        drive.setPoseEstimate(START_POSE);

        TrajectorySequence driveFromPlacedConePoseToParkingPosition;

        switch (aprilTagPipeline.parkingPosition) {
            case ZONE1:
                driveFromPlacedConePoseToParkingPosition = drive.trajectorySequenceBuilder(START_POSE)
                        .splineTo(new Vector2d(-60, -18), Math.toRadians(45))
                        .splineTo(new Vector2d(-36, -12), Math.toRadians(0))
                        .build();
                break;
            default: // ZONE2 and NO_TAGS_SEEN
                driveFromPlacedConePoseToParkingPosition = drive.trajectorySequenceBuilder(START_POSE)
                        .splineTo(new Vector2d(-36, -36), Math.toRadians(0))
                        .build();
                break;
            case ZONE3:
                driveFromPlacedConePoseToParkingPosition = drive.trajectorySequenceBuilder(START_POSE)
                        .splineTo(new Vector2d(-60, -54), Math.toRadians(-45))
                        .splineTo(new Vector2d(-36, -60), Math.toRadians(0))
                        .build();
                break;
        }

        waitForStart();

        drive.followTrajectorySequence(driveFromPlacedConePoseToParkingPosition);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
