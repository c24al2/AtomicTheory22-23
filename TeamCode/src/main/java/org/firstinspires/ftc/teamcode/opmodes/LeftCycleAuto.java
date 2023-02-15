package org.firstinspires.ftc.teamcode.opmodes;

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
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.CameraController;

@Config
@Autonomous(preselectTeleOp = "Drive TeleOp")
public class LeftCycleAuto extends LinearOpMode {
    public static Pose2d START_POSE = new Pose2d(-36, -62.5, Math.toRadians(45));
    public static Pose2d PLACE_CONE_POSE = new Pose2d(-26,  -3.8, Math.toRadians(90));
    public static Pose2d STACK_POSE = new Pose2d(-62.2, -5.2, Math.toRadians(0));

    public static int CONES_TO_PLACE = 2;
    private int placedCones = 0;

    private enum State {
        PLACE_PRELOADED_CONE,
        GO_TO_STACK,
        PICKUP_CONE_FROM_STACK_AND_PLACE,
        PARK,
    }

    State currentState = State.PLACE_PRELOADED_CONE;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        CameraController cameraController = new CameraController(hardwareMap);
        AprilTagPipeline aprilTagPipeline = new AprilTagPipeline();
        cameraController.setPipeline(aprilTagPipeline);

        SampleOmniDrive drive = new SampleOmniDrive(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        drive.setPoseEstimate(START_POSE);

        TrajectorySequence placePreloadedCone = drive.trajectorySequenceBuilder(START_POSE)
                .turn(Math.toRadians(45))
                .addTemporalMarker(() -> intake.setClawPosition(IntakeConstants.CLAW_CLOSED_POSITION))
                .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.HIGH_JUNCTION_HEIGHT))
                .lineToSplineHeading(new Pose2d(-18, -60, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-12, -36), Math.toRadians(90))
                .splineToConstantHeading(PLACE_CONE_POSE.vec(), Math.toRadians(0))
//                .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.HIGH_JUNCTION_HEIGHT - IntakeConstants.ON_JUNCTION_HEIGHT_CHANGE))
//                .waitSeconds(0.2)
                .addTemporalMarker(() -> intake.setClawPosition(IntakeConstants.CLAW_OPEN_POSITION))
//                .waitSeconds(0.1)
//                .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.HIGH_JUNCTION_HEIGHT))
                .build();

        // The claw should be at right height before this trajectory is run
        TrajectorySequence pickUpConeAndPlace = drive.trajectorySequenceBuilder(STACK_POSE)
                .addTemporalMarker(() -> intake.setClawPosition(IntakeConstants.CLAW_CLOSED_POSITION))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.HIGH_JUNCTION_HEIGHT))
                .waitSeconds(0.5)
                .back(35)
                .turn(Math.toRadians(-90))
                .lineTo(PLACE_CONE_POSE.vec())
                .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.HIGH_JUNCTION_HEIGHT - IntakeConstants.ON_JUNCTION_HEIGHT_CHANGE))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> intake.setClawPosition(IntakeConstants.CLAW_OPEN_POSITION))
                .waitSeconds(0.1)
                .build();

        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(PLACE_CONE_POSE)
                .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.STACK_HEIGHTS[5-placedCones-1]))
                .back(5.7)
                .turn(Math.toRadians(90))
                .lineTo(STACK_POSE.vec())
                .build();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Parking Position", aprilTagPipeline.getParkingPosition());
            telemetry.update();
            sleep(25);
        }

        TrajectorySequence driveFromPlacedConePoseToParkingPosition;

        switch (aprilTagPipeline.getParkingPosition()) {
            case ZONE1:
                driveFromPlacedConePoseToParkingPosition = drive.trajectorySequenceBuilder(PLACE_CONE_POSE)
                        .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.BOTTOM))
                        .setReversed(true)
                        .splineTo(new Vector2d(-60, -14), Math.toRadians(180))
                        .build();
                break;
            default: // ZONE2 and NO_TAGS_SEEN
                driveFromPlacedConePoseToParkingPosition = drive.trajectorySequenceBuilder(PLACE_CONE_POSE)
                        .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.BOTTOM))
                        .setReversed(true)
                        .splineTo(new Vector2d(-40, -14), Math.toRadians(270))
                        .build();
                break;
            case ZONE3:
                driveFromPlacedConePoseToParkingPosition = drive.trajectorySequenceBuilder(PLACE_CONE_POSE)
                        .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.BOTTOM))
                        .setReversed(true)
                        .splineTo(new Vector2d(-12, -14), Math.toRadians(0))
                        .build();
                break;
        }

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case PLACE_PRELOADED_CONE:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(placePreloadedCone);
                        currentState = State.GO_TO_STACK;
                    }
                    break;
                case GO_TO_STACK:
                    if (!drive.isBusy()) {
                        if (placedCones == CONES_TO_PLACE) {
                            drive.followTrajectorySequenceAsync(driveFromPlacedConePoseToParkingPosition);
                            currentState = State.PARK;
                        } else {
                            drive.followTrajectorySequenceAsync(goToStack);
                            currentState = State.PICKUP_CONE_FROM_STACK_AND_PLACE;
                            placedCones++;
                        }
                    }
                    break;
                case PICKUP_CONE_FROM_STACK_AND_PLACE:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(pickUpConeAndPlace);
                        currentState = State.GO_TO_STACK;
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                        requestOpModeStop();
                    }
                    break;
            }

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            intake.stepController();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = drive.getPoseEstimate();

            telemetry.update();
        }
    }
}