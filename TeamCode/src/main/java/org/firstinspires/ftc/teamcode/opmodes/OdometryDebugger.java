package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.OdometryLocalizer;

import java.util.List;

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
        List<Double> wheelPositions = odometryLocalizer.getWheelPositions();
        telemetry.addData("Front Encoder Position", wheelPositions.get(0));
        telemetry.addData("Left Encoder Position", wheelPositions.get(1));
        telemetry.addData("Right Encoder Position", wheelPositions.get(2));

        telemetry.addLine();

        List<Double> wheelVelocities = odometryLocalizer.getWheelVelocities();
        telemetry.addData("Front Encoder Velocity", wheelVelocities.get(0));
        telemetry.addData("Left Encoder Velocity", wheelVelocities.get(1));
        telemetry.addData("Right Encoder Velocity", wheelVelocities.get(2));
    }
}
