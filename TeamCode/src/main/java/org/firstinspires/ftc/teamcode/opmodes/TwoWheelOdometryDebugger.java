package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleOmniDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelOdometryLocalizer;

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
        telemetry.addData("Left Encoder Position", twoWheelOdometryLocalizer.leftEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder Position", twoWheelOdometryLocalizer.rightEncoder.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Left Encoder Velocity", twoWheelOdometryLocalizer.leftEncoder.getRawVelocity());
        telemetry.addData("Right Encoder Velocity", twoWheelOdometryLocalizer.rightEncoder.getRawVelocity());
    }
}
