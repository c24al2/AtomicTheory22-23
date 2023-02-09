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
import org.firstinspires.ftc.teamcode.vision.CameraController;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;

@Config
@Autonomous(preselectTeleOp = "Drive TeleOp")
public class RightCycleAuto extends LinearOpMode {
    public static Pose2d START_POSE = new Pose2d(36, -62.5, Math.toRadians(90));
    public static Pose2d STACK_POSE = new Pose2d(60, -12, Math.toRadians(0));
    public static Pose2d PLACE_CONE_POSE = new Pose2d(30, -5, Math.toRadians(135));

    public int CONES_TO_PLACE = 3;

    private enum State {
        PLACE_PRELOADED_CONE_AND_GO_TO_STACK,
        PICKUP_CONE_FROM_STACK_AND_PLACE,
        GO_TO_STACK,
        PARK,
    }

    State currentState = State.PLACE_PRELOADED_CONE_AND_GO_TO_STACK;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        CameraController cameraController = new CameraController(hardwareMap);
        AprilTagPipeline aprilTagPipeline = new AprilTagPipeline();
        cameraController.setPipeline(aprilTagPipeline);

        SampleOmniDrive drive = new SampleOmniDrive(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        drive.setPoseEstimate(START_POSE);

        TrajectorySequence placePreloadedConeAndGoToStack = drive.trajectorySequenceBuilder(START_POSE)
                .addTemporalMarker(() -> intake.setClawPosition(IntakeConstants.CLAW_CLOSED_POSITION))
                .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.HIGH_JUNCTION_HEIGHT))
                .lineToSplineHeading(new Pose2d(18, -60, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(12, -36), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(24, -7), Math.toRadians(0))
                .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.ON_HIGH_JUNCTION_HEIGHT))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> intake.setClawPosition(IntakeConstants.CLAW_OPEN_POSITION))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.GROUND_JUNCTION_HEIGHT))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(STACK_POSE, STACK_POSE.getHeading())
                .build();

        // The claw should ALWAYS be at right height before this trajectory is run
        TrajectorySequence pickUpConeAndPlace = drive.trajectorySequenceBuilder(STACK_POSE)
                .addTemporalMarker(() -> intake.setClawPosition(IntakeConstants.CLAW_CLOSED_POSITION))
                .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.HIGH_JUNCTION_HEIGHT))
                .waitSeconds(0.2)
                .setReversed(true)
                .splineToSplineHeading(PLACE_CONE_POSE, PLACE_CONE_POSE.getHeading())
                .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.ON_HIGH_JUNCTION_HEIGHT))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> intake.setClawPosition(IntakeConstants.CLAW_OPEN_POSITION))
                .waitSeconds(0.1)
                .build();

        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(PLACE_CONE_POSE)
                .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.STACK_HEIGHTS[CONES_TO_PLACE-1]))
                .setReversed(true)
                .splineToSplineHeading(STACK_POSE, STACK_POSE.getHeading())
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
                        .splineTo(new Vector2d(12, -12), Math.toRadians(180))
                        .build();
                break;
            default: // ZONE2 and NO_TAGS_SEEN
                driveFromPlacedConePoseToParkingPosition = drive.trajectorySequenceBuilder(PLACE_CONE_POSE)
                        .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.BOTTOM))
                        .setReversed(true)
                        .splineTo(new Vector2d(36, -12), Math.toRadians(270))
                        .build();
                break;
            case ZONE3:
                driveFromPlacedConePoseToParkingPosition = drive.trajectorySequenceBuilder(PLACE_CONE_POSE)
                        .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.BOTTOM))
                        .setReversed(true)
                        .splineTo(new Vector2d(60, -12), Math.toRadians(0))
                        .build();
                break;
        }

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case PLACE_PRELOADED_CONE_AND_GO_TO_STACK:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(placePreloadedConeAndGoToStack);
                        currentState = State.PICKUP_CONE_FROM_STACK_AND_PLACE;
                    }
                    break;
                case PICKUP_CONE_FROM_STACK_AND_PLACE:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(pickUpConeAndPlace);
                        currentState = State.GO_TO_STACK;
                    }
                    break;
                case GO_TO_STACK:
                    if (!drive.isBusy()) {
                        if (CONES_TO_PLACE == 0) {
                            drive.followTrajectorySequenceAsync(driveFromPlacedConePoseToParkingPosition);
                            currentState = State.PARK;
                        } else {
                            drive.followTrajectorySequenceAsync(goToStack);
                            currentState = State.PICKUP_CONE_FROM_STACK_AND_PLACE;
                            CONES_TO_PLACE--;
                        }
                    }
                    break;
                case PARK:
                    // currentState does not change once in PARK
                    // This concludes the autonomous program
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