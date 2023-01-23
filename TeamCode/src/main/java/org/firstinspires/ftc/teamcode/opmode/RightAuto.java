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
import org.firstinspires.ftc.teamcode.vision.CameraController;
import org.firstinspires.ftc.teamcode.vision.ParkingPositionPipeline;

@Config
@Autonomous
public class RightAuto extends LinearOpMode {
    public static Pose2d START_POSE = new Pose2d(36, -62.8, Math.toRadians(90));
    public static Pose2d PLACE_PRELOADED_CONE_POSE = new Pose2d(18, -7, Math.toRadians(45));
    public static Pose2d PICKUP_CONE_FROM_STACK_POSE = new Pose2d(60, -12, Math.toRadians(0));
    public static Pose2d PLACE_STACK_CONE_POSE = new Pose2d(31, -7, Math.toRadians(135));

    // This is essentially just defines the possible steps our program will take
    enum State {
        START,
        PLACE_PRELOADED_CONE,
        PRELOADED_CONE_TO_STACK,
        PICKUP_CONE_FROM_STACK_AND_PLACE,
        GO_TO_STACK,
        DONE,
    }

    State currentState = State.START;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        CameraController cameraController = new CameraController(hardwareMap);
        ParkingPositionPipeline aprilTagPipeline = new ParkingPositionPipeline();
        cameraController.setPipeline(aprilTagPipeline);

        telemetry.addData("Parking Position", aprilTagPipeline.parkingPosition);
        telemetry.update();

        cameraController.stopStreaming(); // Reduce resource usage by not processing any more camera

        SampleOmniDrive drive = new SampleOmniDrive(hardwareMap);
        drive.setPoseEstimate(START_POSE);

        TrajectorySequence placePreloadedCone = drive.trajectorySequenceBuilder(START_POSE)
                .lineToSplineHeading(new Pose2d(18, -60, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(12, -36), Math.toRadians(90))
                .splineTo(PLACE_PRELOADED_CONE_POSE.vec(), PLACE_PRELOADED_CONE_POSE.getHeading())
                .build();

        TrajectorySequence driveFromPlacedPreloadedConePoseToStack = drive.trajectorySequenceBuilder(PLACE_PRELOADED_CONE_POSE)
                .setReversed(true)
                .splineToLinearHeading(PICKUP_CONE_FROM_STACK_POSE, PICKUP_CONE_FROM_STACK_POSE.getHeading())
                .build();

        // The claw should ALWAYS be at right height before this trajectory is run
        TrajectorySequence pickUpConeAndPlace = drive.trajectorySequenceBuilder(PICKUP_CONE_FROM_STACK_POSE)
                .setReversed(true)
                .splineToSplineHeading(PLACE_STACK_CONE_POSE, PLACE_STACK_CONE_POSE.getHeading())
                .build();

        TrajectorySequence driveFromPlacedConePoseToStack = drive.trajectorySequenceBuilder(PLACE_STACK_CONE_POSE)
                .setReversed(true)
                .splineToSplineHeading(PICKUP_CONE_FROM_STACK_POSE, PICKUP_CONE_FROM_STACK_POSE.getHeading())
                .build();

        TrajectorySequence driveFromPlacedConePoseToParkingPosition;

        switch (aprilTagPipeline.parkingPosition) {
            case ZONE1:
                driveFromPlacedConePoseToParkingPosition = drive.trajectorySequenceBuilder(PLACE_STACK_CONE_POSE)
                        .setReversed(true)
                        .splineTo(new Vector2d(18, -12), Math.toRadians(180))
                        .splineTo(new Vector2d(12, -36), Math.toRadians(270))
                        .build();
                break;
            default: // ZONE2 and NO_TAGS_SEEN
                driveFromPlacedConePoseToParkingPosition = drive.trajectorySequenceBuilder(PLACE_STACK_CONE_POSE)
                        .setReversed(true)
                        .splineTo(new Vector2d(36, -36), Math.toRadians(270))
                        .build();
                break;
            case ZONE3:
                driveFromPlacedConePoseToParkingPosition = drive.trajectorySequenceBuilder(PLACE_STACK_CONE_POSE)
                        .setReversed(true)
                        .splineTo(new Vector2d(54, -12), Math.toRadians(0))
                        .splineTo(new Vector2d(60, -36), Math.toRadians(270))
                        .build();
                break;
        }

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case START:
                    if (!drive.isBusy()) {
                        currentState = State.PLACE_PRELOADED_CONE;
                        drive.followTrajectorySequenceAsync(placePreloadedCone);
                    }
                    break;
                case PLACE_PRELOADED_CONE:
                    if (!drive.isBusy()) {
                        currentState = State.PRELOADED_CONE_TO_STACK;
                        drive.followTrajectorySequenceAsync(driveFromPlacedPreloadedConePoseToStack);
                    }
                    break;
                case PRELOADED_CONE_TO_STACK:
                    if (!drive.isBusy()) {
                        currentState = State.PICKUP_CONE_FROM_STACK_AND_PLACE;
                        drive.followTrajectorySequenceAsync(pickUpConeAndPlace);
                    }
                    break;
                case PICKUP_CONE_FROM_STACK_AND_PLACE:
                    if (!drive.isBusy()) {
                        currentState = State.DONE;
                    }
                    break;
                case DONE:
                    // Do nothing in DONE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
//            lift.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
