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
public class RightAuto extends LinearOpMode {
    // These can be changed using FTC Dashboard!
    public static Pose2d START_POSE = new Pose2d(-62.8, -35, Math.toRadians(0));
    public static Pose2d PLACE_PRELOADED_CONE_POSE = new Pose2d(-1, -17, Math.toRadians(-46));
    public static Pose2d PICKUP_CONE_FROM_STACK_POSE = new Pose2d(-8, -64, Math.toRadians(-90));
    public static Pose2d PLACE_STACK_CONE_POSE = new Pose2d(-1, -31, Math.toRadians(46));

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

        TrajectorySequence placePreloadedCone = drive.trajectorySequenceBuilder(START_POSE)
//                .addTemporalMarker(() -> liftandServo.intakeFullStep(DriveConstants.GROUNDJUNCTION))
                .waitSeconds(0.1)
                .lineToSplineHeading(new Pose2d(-60, -20, 0))
                .splineToConstantHeading(new Vector2d(-35, -12), 0)
                .splineTo(new Vector2d(-22, -12), 0)
                .splineTo(PLACE_PRELOADED_CONE_POSE.vec(), PLACE_PRELOADED_CONE_POSE.getHeading())
//                .addTemporalMarker(() -> liftandServo.intakeFullStep(DriveConstants.HIGHJUNCTION))
//                .addTemporalMarker(() -> liftandServo.clawOpen())
                .build();

        TrajectorySequence driveFromPlacedPreloadedConePoseToStack = drive.trajectorySequenceBuilder(PLACE_PRELOADED_CONE_POSE)
//                .addTemporalMarker(() -> liftandServo.intakeFullStep(DriveConstants.PICKUP_CONE_1))
                .setReversed(true)
                .splineTo(new Vector2d(-8, -12), Math.toRadians(90))
                .setReversed(false)
                .splineTo(PICKUP_CONE_FROM_STACK_POSE.vec(), PICKUP_CONE_FROM_STACK_POSE.getHeading())
                .build();

        // The claw should ALWAYS be at right height before this trajectory is run
        TrajectorySequence pickUpConeAndPlace = drive.trajectorySequenceBuilder(PICKUP_CONE_FROM_STACK_POSE)
//                .addTemporalMarker(() -> liftandServo.clawClose())
//                .addTemporalMarker(() -> liftandServo.intakeFullStep(DriveConstants.MEDIUMJUNCTION))
                .waitSeconds(0.1)
                .setReversed(true)
                .splineToSplineHeading(PLACE_STACK_CONE_POSE, PLACE_STACK_CONE_POSE.getHeading())
//                .addTemporalMarker(() -> liftandServo.intakeFullStep(DriveConstants.HIGHJUNCTION))
//                .addTemporalMarker(() -> liftandServo.clawOpen())
                .waitSeconds(0.2)
                .build();

        // Takes 2.57s
        TrajectorySequence driveFromPlacedConePoseToStack = drive.trajectorySequenceBuilder(PLACE_STACK_CONE_POSE)
//                .addTemporalMarker(() -> liftandServo.intakeFullStep(DriveConstants.PICKUP_CONE_2))
                .setReversed(true)
                .splineToSplineHeading(PICKUP_CONE_FROM_STACK_POSE, PICKUP_CONE_FROM_STACK_POSE.getHeading())
                .build();

        TrajectorySequence driveFromPlacedConePoseToParkingPosition;

        switch (aprilTagPipeline.parkingPosition) {
            case ZONE1:
                // TODO: Optimize Zone1 parking
                driveFromPlacedConePoseToParkingPosition = drive.trajectorySequenceBuilder(PLACE_STACK_CONE_POSE)
                        .lineToSplineHeading(new Pose2d(-12, -24, Math.toRadians(90)))
                        .splineTo(new Vector2d(-32, -12), Math.toRadians(180))
                        .build();
                break;
            default: // ZONE2 and NO_TAGS_SEEN
                driveFromPlacedConePoseToParkingPosition = drive.trajectorySequenceBuilder(PLACE_STACK_CONE_POSE)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(-32, -36, Math.toRadians(180)), Math.toRadians(180))
                        .build();
                break;
            case ZONE3:
                driveFromPlacedConePoseToParkingPosition = drive.trajectorySequenceBuilder(PLACE_STACK_CONE_POSE)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(-12, -48, Math.toRadians(-90)), Math.toRadians(-90))
                        .splineTo(new Vector2d(-32, -60), Math.toRadians(180))
                        .build();
                break;
        }

        waitForStart();

        drive.followTrajectorySequence(placePreloadedCone);
        drive.followTrajectorySequence(driveFromPlacedPreloadedConePoseToStack);

        drive.followTrajectorySequence(pickUpConeAndPlace);
        drive.followTrajectorySequence(driveFromPlacedConePoseToStack);
        drive.followTrajectorySequence(pickUpConeAndPlace);
        drive.followTrajectorySequence(driveFromPlacedConePoseToStack);
        drive.followTrajectorySequence(pickUpConeAndPlace);
//        drive.followTrajectorySequence(driveFromPlacedConePoseToStack);
//        drive.followTrajectorySequence(pickUpConeAndPlace);

        drive.followTrajectorySequence(driveFromPlacedConePoseToParkingPosition);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
