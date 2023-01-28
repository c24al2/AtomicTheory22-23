package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.CameraController;
import org.firstinspires.ftc.teamcode.vision.ParkingPositionPipeline;

@TeleOp
public class AprilTagDebugger extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        CameraController cameraController = new CameraController(hardwareMap);
        ParkingPositionPipeline aprilTagPipeline = new ParkingPositionPipeline();

        waitForStart();
        if (isStopRequested()) return;

        cameraController.setPipeline(aprilTagPipeline);

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Parking Position", aprilTagPipeline.parkingPosition);
            telemetry.update();
        }
    }
}