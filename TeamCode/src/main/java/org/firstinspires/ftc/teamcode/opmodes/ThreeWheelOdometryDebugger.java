package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleOmniDrive;
import org.firstinspires.ftc.teamcode.drive.ThreeWheelOdometryLocalizer;

import java.util.List;

@Config
@TeleOp(group = "debug")
public class ThreeWheelOdometryDebugger extends OpMode {
    public ThreeWheelOdometryLocalizer threeWheelOdometryLocalizer;
    public SampleOmniDrive drive;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        threeWheelOdometryLocalizer = new ThreeWheelOdometryLocalizer(hardwareMap);
    }

    @Override
    public void loop() {
        List<Double> encoderPositions = threeWheelOdometryLocalizer.getWheelPositions();
        telemetry.addData("Front Encoder Position (in)", encoderPositions.get(0));
        telemetry.addData("Left Encoder Position (in)", encoderPositions.get(1));
        telemetry.addData("Right Encoder Position (in)", encoderPositions.get(2));

        telemetry.addLine();

        List<Double> encoderVelocities = threeWheelOdometryLocalizer.getWheelVelocities();
        telemetry.addData("Front Encoder Velocity (in/s)", encoderVelocities.get(0));
        telemetry.addData("Left Encoder Velocity (in/s)", encoderVelocities.get(1));
        telemetry.addData("Right Encoder Velocity (in/s)", encoderVelocities.get(2));
    }
}
