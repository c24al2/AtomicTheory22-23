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
public class ParkingAuto extends LinearOpMode {
    public static Pose2d START_POSE = new Pose2d(36, -62.8, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        CameraController cameraController = new CameraController(hardwareMap);
        AprilTagPipeline aprilTagPipeline = new AprilTagPipeline();
        cameraController.setPipeline(aprilTagPipeline);

        SampleOmniDrive drive = new SampleOmniDrive(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        drive.setPoseEstimate(START_POSE);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Parking Position", aprilTagPipeline.getParkingPosition());
            telemetry.update();
            sleep(25);
        }

        waitForStart();
        if (isStopRequested()) return;

        intake.setClawPosition(IntakeConstants.CLAW_CLOSED_POSITION);
        intake.followMotionProfile(IntakeConstants.MEDIUM_JUNCTION_HEIGHT);

        TrajectorySequence driveToParkingPosition;

        switch (aprilTagPipeline.getParkingPosition()) {
            case ZONE1:
                driveToParkingPosition = drive.trajectorySequenceBuilder(START_POSE)
                        .lineToSplineHeading(new Pose2d(18, -60, Math.toRadians(90)))
                        .splineToConstantHeading(new Vector2d(12, -24), Math.toRadians(90))
                        .build();
                break;
            default: // ZONE2 and NO_TAGS_SEEN
                driveToParkingPosition = drive.trajectorySequenceBuilder(START_POSE)
                        .splineTo(new Vector2d(36, -24), Math.toRadians(90))
                        .build();
                break;
            case ZONE3:
                driveToParkingPosition = drive.trajectorySequenceBuilder(START_POSE)
                        .lineToSplineHeading(new Pose2d(54, -60, Math.toRadians(90)))
                        .splineToConstantHeading(new Vector2d(60, -24), Math.toRadians(90))
                        .build();
                break;
        }

        drive.followTrajectorySequence(driveToParkingPosition);

        intake.followMotionProfile(IntakeConstants.BOTTOM);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}