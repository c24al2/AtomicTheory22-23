package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.CameraController;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;

@Config
@TeleOp(group = "debug")
public class AprilTagDebugger extends OpMode {
    public CameraController cameraController;
    public AprilTagPipeline aprilTagPipeline;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        cameraController = new CameraController(hardwareMap);
        aprilTagPipeline = new AprilTagPipeline();
        cameraController.setPipeline(aprilTagPipeline);
    }

    @Override
    public void loop() {
        telemetry.addData("Parking Position", aprilTagPipeline.getParkingPosition());
    }
}