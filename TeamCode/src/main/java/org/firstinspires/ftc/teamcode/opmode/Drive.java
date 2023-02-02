package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleOmniDrive;
import org.firstinspires.ftc.teamcode.intake.IntakeConstants;


@Config
@TeleOp
public class Drive extends OpMode {
    public static double SLOW_MODE_SCALAR = 0.4;
    public static double LIFT_MULTIPLIER = 1000; // TICKS/SEC. This num should be pretty close to MAX_VEL? If it's too far off, then when we stop moving the stick the lift will continue moving for a while after

    // TODO: Remove, this is for debugging purpose
    // private static Pose2d START_POSE = PoseStorage.currentPose;
    private static Pose2d START_POSE = new Pose2d(36, -62.8, Math.toRadians(90));

    private ElapsedTime timer;

    private boolean driverSlowMode = false;
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();

    public SampleOmniDrive drive;
    public Intake intake;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        timer = new ElapsedTime();

        drive = new SampleOmniDrive(hardwareMap);
        intake = new Intake(hardwareMap);

        drive.setPoseEstimate(START_POSE);
    }

    @Override
    public void loop() {
       double servoTarget = 0;

        // Enable toggling driver slow mode when press a
        if (!previousGamepad1.a && gamepad1.a) {
            driverSlowMode = !driverSlowMode;
        }

        // Create a vector from the gamepad x/y inputs
        Vector2d translationalInput = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);

        // Read pose
//        Pose2d poseEstimate = drive.getPoseEstimate();
        // Then, rotate that vector by the inverse of the robots' DELTA heading for driver lock
//        translationalInput = translationalInput.rotated(-(poseEstimate.getHeading() - START_POSE.getHeading()));

        Pose2d input = new Pose2d(translationalInput, -gamepad1.right_stick_x);

        if (driverSlowMode) {
            input = input.times(SLOW_MODE_SCALAR);
        }

        // Pass in the rotated input + right stick value for rotation
        drive.setWeightedDrivePower(input);

        double deltaPosition = -gamepad2.left_stick_y * LIFT_MULTIPLIER * timer.time();
        telemetry.addData("deltaPosition", deltaPosition);
        intake.setRelativeTargetPosition(deltaPosition);
        timer.reset();

        if (!previousGamepad2.dpad_up && gamepad2.dpad_up) {
            intake.setTargetPosition(IntakeConstants.HIGH_JUNCTION_HEIGHT);
        }

        if (!previousGamepad2.dpad_right && gamepad2.dpad_right) {
            intake.setTargetPosition(IntakeConstants.MEDIUM_JUNCTION_HEIGHT);
        }

        if (!previousGamepad2.dpad_left && gamepad2.dpad_left) {
            intake.setTargetPosition(IntakeConstants.LOW_JUNCTION_HEIGHT);
        }

        if (!previousGamepad2.dpad_down && gamepad2.dpad_down) {
            intake.setTargetPosition(IntakeConstants.GROUND_JUNCTION_HEIGHT);
        }

        drive.update();
        intake.followMotionProfile();

        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
