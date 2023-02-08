package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.OdometryLocalizer;

@Config
@TeleOp(group = "debug")
public class OdometryDebugger extends OpMode {
    public OdometryLocalizer odometryLocalizer;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        odometryLocalizer = new OdometryLocalizer(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Front Encoder Position", odometryLocalizer.frontEncoder.getCurrentPosition());
        telemetry.addData("Left Encoder Position", odometryLocalizer.leftEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder Position", odometryLocalizer.rightEncoder.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Front Encoder Velocity", odometryLocalizer.frontEncoder.getRawVelocity());
        telemetry.addData("Left Encoder Velocity", odometryLocalizer.leftEncoder.getRawVelocity());
        telemetry.addData("Right Encoder Velocity", odometryLocalizer.rightEncoder.getRawVelocity());
    }
}
