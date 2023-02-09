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
@Config
@TeleOp(group = "debug")
public class TwoWheelOdometryDebugger extends OpMode {
    public TwoWheelOdometryLocalizer twoWheelOdometryLocalizer;
    public SampleOmniDrive drive;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleOmniDrive drive = new SampleOmniDrive(hardwareMap);
        twoWheelOdometryLocalizer = new TwoWheelOdometryLocalizer(hardwareMap, drive);
    }

    @Override
    public void loop() {
        List<Double> encoderPositions = twoWheelOdometryLocalizer.getWheelPositions();
        telemetry.addData("Left Encoder Position (in)", encoderPositions.get(0));
        telemetry.addData("Right Encoder Position (in)", encoderPositions.get(1));

        telemetry.addLine();

        List<Double> encoderVelocities = twoWheelOdometryLocalizer.getWheelVelocities();
        telemetry.addData("Left Encoder Velocity (in/s)", encoderVelocities.get(0));
        telemetry.addData("Right Encoder Velocity (in/s)", encoderVelocities.get(1));
    }
}
