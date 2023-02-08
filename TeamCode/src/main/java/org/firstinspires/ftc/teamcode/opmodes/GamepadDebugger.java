package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.CameraController;

@Config
@TeleOp(group = "debug")
public class GamepadDebugger extends OpMode {
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        telemetry.addData("Gamepad 1 Left Stick X", -gamepad1.left_stick_x);
        telemetry.addData("Gamepad 1 Left Stick Y", -gamepad1.left_stick_y);
        telemetry.addData("Gamepad 1 Right Stick X", -gamepad1.right_stick_x);
        telemetry.addData("Gamepad 1 Right Stick Y", -gamepad1.right_stick_y);
        telemetry.addLine();
        telemetry.addData("Gamepad 2 Left Stick X", -gamepad2.left_stick_x);
        telemetry.addData("Gamepad 2 Left Stick Y", -gamepad2.left_stick_y);
        telemetry.addData("Gamepad 2 Right Stick X", -gamepad2.right_stick_x);
        telemetry.addData("Gamepad 2 Right Stick Y", -gamepad2.right_stick_y);
    }
}