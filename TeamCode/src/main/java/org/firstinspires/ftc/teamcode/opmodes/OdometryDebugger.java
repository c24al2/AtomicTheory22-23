package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ThreeWheelOdometryLocalizer;

@Config
@TeleOp(group = "debug")
public class OdometryDebugger extends OpMode {
    public ThreeWheelOdometryLocalizer threeWheelOdometryLocalizer;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        threeWheelOdometryLocalizer = new ThreeWheelOdometryLocalizer(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Front Encoder Position", threeWheelOdometryLocalizer.frontEncoder.getCurrentPosition());
        telemetry.addData("Left Encoder Position", threeWheelOdometryLocalizer.leftEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder Position", threeWheelOdometryLocalizer.rightEncoder.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Front Encoder Velocity", threeWheelOdometryLocalizer.frontEncoder.getRawVelocity());
        telemetry.addData("Left Encoder Velocity", threeWheelOdometryLocalizer.leftEncoder.getRawVelocity());
        telemetry.addData("Right Encoder Velocity", threeWheelOdometryLocalizer.rightEncoder.getRawVelocity());
    }
}
