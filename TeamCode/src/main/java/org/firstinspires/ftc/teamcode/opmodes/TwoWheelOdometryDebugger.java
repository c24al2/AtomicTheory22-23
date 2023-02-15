package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleOmniDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelOdometryLocalizer;

import java.util.List;

@Disabled
@TeleOp(group = "debug")
public class TwoWheelOdometryDebugger extends OpMode {
    public TwoWheelOdometryLocalizer twoWheelOdometryLocalizer;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleOmniDrive drive = new SampleOmniDrive(hardwareMap);
        twoWheelOdometryLocalizer = new TwoWheelOdometryLocalizer(hardwareMap, drive);
    }

    @Override
    public void loop() {
        telemetry.addData("Left Encoder Position (ticks)", twoWheelOdometryLocalizer.leftEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder Position (ticks)", twoWheelOdometryLocalizer.rightEncoder.getCurrentPosition());

        telemetry.addLine();

        List<Double> encoderPositions = twoWheelOdometryLocalizer.getWheelPositions();
        telemetry.addData("Left Encoder Position (in)", encoderPositions.get(0));
        telemetry.addData("Right Encoder Position (in)", encoderPositions.get(1));

        telemetry.addLine();

        telemetry.addData("Left Encoder Velocity (ticks/s)", twoWheelOdometryLocalizer.leftEncoder.getRawVelocity());
        telemetry.addData("Right Encoder Velocity (ticks/s)", twoWheelOdometryLocalizer.rightEncoder.getRawVelocity());

        telemetry.addLine();

        List<Double> encoderVelocities = twoWheelOdometryLocalizer.getWheelVelocities();
        telemetry.addData("Left Encoder Velocity (in/s)", encoderVelocities.get(0));
        telemetry.addData("Right Encoder Velocity (in/s)", encoderVelocities.get(1));
    }
}