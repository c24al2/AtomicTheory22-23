package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ThreeWheelOdometryLocalizer;

import java.util.List;

@Disabled
@TeleOp(group = "debug")
public class ThreeWheelOdometryDebugger extends OpMode {
    public ThreeWheelOdometryLocalizer threeWheelOdometryLocalizer;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        threeWheelOdometryLocalizer = new ThreeWheelOdometryLocalizer(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Front Encoder Position (ticks)", threeWheelOdometryLocalizer.frontEncoder.getCurrentPosition());
        telemetry.addData("Left Encoder Position (ticks)", threeWheelOdometryLocalizer.leftEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder Position (ticks)", threeWheelOdometryLocalizer.rightEncoder.getCurrentPosition());

        telemetry.addLine();

        List<Double> encoderPositions = threeWheelOdometryLocalizer.getWheelPositions();
        telemetry.addData("Front Encoder Position (in)", encoderPositions.get(0));
        telemetry.addData("Left Encoder Position (in)", encoderPositions.get(1));
        telemetry.addData("Right Encoder Position (in)", encoderPositions.get(2));

        telemetry.addLine();

        telemetry.addData("Front Encoder Velocity (ticks/s)", threeWheelOdometryLocalizer.frontEncoder.getRawVelocity());
        telemetry.addData("Left Encoder Velocity (ticks/s)", threeWheelOdometryLocalizer.leftEncoder.getRawVelocity());
        telemetry.addData("Right Encoder Velocity (ticks/s)", threeWheelOdometryLocalizer.rightEncoder.getRawVelocity());

        telemetry.addLine();

        List<Double> encoderVelocities = threeWheelOdometryLocalizer.getWheelVelocities();
        telemetry.addData("Front Encoder Velocity (in/s)", encoderVelocities.get(0));
        telemetry.addData("Left Encoder Velocity (in/s)", encoderVelocities.get(1));
        telemetry.addData("Right Encoder Velocity (in/s)", encoderVelocities.get(2));
    }
}
