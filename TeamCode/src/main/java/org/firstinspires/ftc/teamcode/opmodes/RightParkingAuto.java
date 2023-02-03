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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.CameraController;

@Config
@Autonomous
public class RightParkingAuto extends LinearOpMode {
    public static Pose2d START_POSE = new Pose2d(36, -62.8, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        CameraController cameraController = new CameraController(hardwareMap);
        AprilTagPipeline aprilTagPipeline = new AprilTagPipeline();
        cameraController.setPipeline(aprilTagPipeline);

        telemetry.addData("Parking Position", aprilTagPipeline.getParkingPosition());
        telemetry.update();

        cameraController.stopStreaming(); // Reduce resource usage by not processing any more camera

        SampleOmniDrive drive = new SampleOmniDrive(hardwareMap);

        drive.setPoseEstimate(START_POSE);

        TrajectorySequence driveToParkingPosition;

        switch (aprilTagPipeline.getParkingPosition()) {
            case ZONE1:
                driveToParkingPosition = drive.trajectorySequenceBuilder(START_POSE)
                        .lineToSplineHeading(new Pose2d(18, -60, Math.toRadians(90)))
                        .splineToConstantHeading(new Vector2d(12, -36), Math.toRadians(90))
                        .build();
                break;
            default: // ZONE2 and NO_TAGS_SEEN
                driveToParkingPosition = drive.trajectorySequenceBuilder(START_POSE)
                        .splineTo(new Vector2d(36, -36), Math.toRadians(90))
                        .build();
                break;
            case ZONE3:
                driveToParkingPosition = drive.trajectorySequenceBuilder(START_POSE)
                        .lineToSplineHeading(new Pose2d(54, -60, Math.toRadians(90)))
                        .splineToConstantHeading(new Vector2d(60, -36), Math.toRadians(90))
                        .build();
                break;
        }

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectorySequenceAsync(driveToParkingPosition);

        while (opModeIsActive() && !isStopRequested()) {
            if (!drive.isBusy()) {
                break;
            }

            // We update drive continuously in the background, regardless of state
            drive.update();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
    }
}