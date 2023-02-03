package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleOmniDrive;

@Config
@TeleOp
public class MotorDebugger extends OpMode {
    public static double MOTOR_POWER = 1;

    public SampleOmniDrive drive;
    public Intake intake;

    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleOmniDrive(hardwareMap);
        intake = new Intake(hardwareMap);

        telemetry.addLine("Press play to begin the motor debugging opmode");
    }

    @Override
    public void loop() {
        telemetry.addLine("Press each button to turn on its respective motor");

        if (gamepad1.x || gamepad2.x) {
            drive.setMotorPowers(MOTOR_POWER, 0, 0);
            telemetry.addData("Running Motor", "Left");
        } else if (gamepad1.a || gamepad2.a) {
            drive.setMotorPowers(0, MOTOR_POWER, 0);
            telemetry.addData("Running Motor", "Back");
        } else if (gamepad1.b || gamepad2.b) {
            drive.setMotorPowers(0, 0, MOTOR_POWER);
            telemetry.addData("Running Motor", "Right");
        } else if (gamepad1.y || gamepad2.y) {
            intake.setPower(0.1);
            telemetry.addData("Running Motor", "Intake");
        } else {
            drive.setMotorPowers(0, 0, 0);
            intake.setPower(0);
            telemetry.addData("Running Motor", "None");
        }

        telemetry.addLine();
        telemetry.addData("Left Motor Position: ", drive.leftMotor.getCurrentPosition());
        telemetry.addData("Back Motor Position: ", drive.backMotor.getCurrentPosition());
        telemetry.addData("Right Motor Position: ", drive.rightMotor.getCurrentPosition());
        telemetry.addData("Intake Motor Position: ", intake.intake.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Left Motor Velocity: ", drive.leftMotor.getVelocity());
        telemetry.addData("Back Motor Velocity: ", drive.backMotor.getVelocity());
        telemetry.addData("Right Motor Velocity: ", drive.rightMotor.getVelocity());
        telemetry.addData("Intake Motor Velocity: ", intake.intake.getVelocity());

        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
