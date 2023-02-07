package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleOmniDrive;
import org.firstinspires.ftc.teamcode.intake.IntakeConstants;


@Config
@TeleOp
public class IterativeOpmode extends OpMode {
    public static double DRIVER_SPEED_SCALAR = 0.73;
    public static double DRIVER_ROTATION_SCALAR = 0.09;
    public static double DRIVER_SLOW_MODE_SCALAR = 0.6;
    public static double DRIVER_CANCEL_SPRINT_THRESHOLD = 0.85;

    public static double INTAKE_SPEED_SCALAR = 0.4;
    public static double GUNNER_STICK_THRESHOLD = 0.025;

    private boolean driverSprintMode = false;
    private boolean driverSlowMode = false;
    public Gamepad previousGamepad1 = new Gamepad();
    public Gamepad previousGamepad2 = new Gamepad();

    public SampleOmniDrive drive;
    public Intake intake;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleOmniDrive(hardwareMap);
        intake = new Intake(hardwareMap);

        drive.setPoseEstimate(PoseStorage.currentPose);
    }

    @Override
    public void loop() {
        if (!previousGamepad1.left_stick_button && gamepad1.left_stick_button) {
            driverSprintMode = true;
        }

        if (-gamepad1.left_stick_y < DRIVER_CANCEL_SPRINT_THRESHOLD) {
            driverSprintMode = false;
        }

        if (!previousGamepad1.a && gamepad1.a) {
            driverSlowMode = !driverSlowMode;
        }

        telemetry.addData("Sprint mode", driverSprintMode);
        telemetry.addData("Slow mode", driverSlowMode);

        Vector2d translationalInput = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);

        if (!driverSprintMode) {
            translationalInput = translationalInput.times(DRIVER_SPEED_SCALAR);
        }

        Pose2d input = new Pose2d(translationalInput, -gamepad1.right_stick_x * DRIVER_ROTATION_SCALAR);

        if (driverSlowMode && !driverSprintMode) {
            input = input.times(DRIVER_SLOW_MODE_SCALAR);
        }

        // Pass in the rotated input + right stick value for rotation
        drive.setWeightedDrivePower(input);

        if (Math.abs(gamepad2.left_stick_y) > GUNNER_STICK_THRESHOLD) {
            intake.setPower(-gamepad2.left_stick_y * INTAKE_SPEED_SCALAR);
        } else {
            intake.stepController();
        }

        if (!previousGamepad2.x && gamepad2.x) {
            intake.setClawPosition(IntakeConstants.CLAW_CLOSED_POSITION);
        }

        if (!previousGamepad2.y && gamepad2.y) {
            intake.setClawPosition(IntakeConstants.CLAW_OPEN_POSITION);
        }

        if (!previousGamepad2.dpad_up && gamepad2.dpad_up) {
            intake.followMotionProfileAsync(IntakeConstants.HIGH_JUNCTION_HEIGHT);
        }

        if (!previousGamepad2.dpad_right && gamepad2.dpad_right) {
            intake.followMotionProfileAsync(IntakeConstants.MEDIUM_JUNCTION_HEIGHT);
        }

        if (!previousGamepad2.dpad_left && gamepad2.dpad_left) {
            intake.followMotionProfileAsync(IntakeConstants.LOW_JUNCTION_HEIGHT);
        }

        if (!previousGamepad2.dpad_down && gamepad2.dpad_down) {
            intake.followMotionProfileAsync(IntakeConstants.GROUND_JUNCTION_HEIGHT);
        }

        drive.update();

        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
