package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleOmniDrive;

import java.util.List;

@Config
@TeleOp(group = "debug")
public class MotorDebugger extends OpMode {
    public static double MOTOR_POWER = 1;

    public SampleOmniDrive drive;
    public Intake intake;

    public Gamepad previousGamepad1 = new Gamepad();
    public Gamepad previousGamepad2 = new Gamepad();

    public double liftPosition = 900;
    public double servoPosition = 0.5;
    public static double LIFT_POSITION_STEP = 100;
    public static double SERVO_POSITION_STEP = 0.025;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleOmniDrive(hardwareMap);
        intake = new Intake(hardwareMap);

        intake.followMotionProfileAsync(liftPosition);
        intake.clawServo.setPosition(servoPosition);
    }

    @Override
    public void loop() {
        if (gamepad1.x || gamepad2.x) {
            drive.setMotorPowers(MOTOR_POWER, 0, 0);
        } else if (gamepad1.a || gamepad2.a) {
            drive.setMotorPowers(0, MOTOR_POWER, 0);
        } else if (gamepad1.b || gamepad2.b) {
            drive.setMotorPowers(0, 0, MOTOR_POWER);
        } else {
            drive.setMotorPowers(0, 0, 0);
        }

        if ((!previousGamepad1.dpad_up && gamepad1.dpad_up) || (!previousGamepad2.dpad_up && gamepad2.dpad_up)) {
            liftPosition += LIFT_POSITION_STEP;
            intake.followMotionProfileAsync(liftPosition);
        }

        if ((!previousGamepad1.dpad_down && gamepad1.dpad_down) || (!previousGamepad2.dpad_down && gamepad2.dpad_down)) {
            liftPosition -= LIFT_POSITION_STEP;
            intake.followMotionProfileAsync(liftPosition);
        }

        if ((!previousGamepad1.dpad_right && gamepad1.dpad_right) || (!previousGamepad2.dpad_right && gamepad2.dpad_right)) {
            servoPosition += SERVO_POSITION_STEP;
            intake.clawServo.setPosition(servoPosition);
        }

        if ((!previousGamepad1.dpad_left && gamepad1.dpad_left) || (!previousGamepad2.dpad_left && gamepad2.dpad_left)) {
            servoPosition -= SERVO_POSITION_STEP;
            intake.clawServo.setPosition(servoPosition);
        }

        intake.stepController();

        List<Double> motorPositions = drive.getWheelPositions();
        telemetry.addData("Left Motor Position (in)", motorPositions.get(0));
        telemetry.addData("Back Motor Position (in)", motorPositions.get(1));
        telemetry.addData("Right Motor Position (in)", motorPositions.get(2));
        telemetry.addData("Intake Motor Position (ticks)", intake.intake.getCurrentPosition());
        telemetry.addData("Intake Servo Position (scalar)", intake.clawServo.getPosition());

        telemetry.addLine();

        List<Double> motorVelocities = drive.getWheelVelocities();
        telemetry.addData("Left Motor Velocity (in/s)", motorVelocities.get(0));
        telemetry.addData("Back Motor Velocity (in/s)", motorVelocities.get(1));
        telemetry.addData("Right Motor Velocity (in/s)", motorVelocities.get(2));
        telemetry.addData("Intake Motor Velocity (ticks/s)", intake.intake.getVelocity());

        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
