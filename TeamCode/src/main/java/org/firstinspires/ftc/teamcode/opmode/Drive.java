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

        if (gamepad2.x){
            intake.closeClaw();
            telemetry.addData("Claw","Claw Closed");
        }

        if(gamepad2.y){
            intake.openClaw();
            telemetry.addData("Claw","Claw Open");
        }

        if (gamepad2.dpad_up) {
            intake.setTargetPosition(IntakeConstants.HIGH_JUNCTION_HEIGHT);
        }

        if (gamepad2.dpad_right) {
            intake.setTargetPosition(IntakeConstants.MEDIUM_JUNCTION_HEIGHT);
        }

        if (gamepad2.dpad_left) {
            intake.setTargetPosition(IntakeConstants.LOW_JUNCTION_HEIGHT);
        }

        if (gamepad2.dpad_down) {
            intake.setTargetPosition(IntakeConstants.GROUND_JUNCTION_HEIGHT);
        }

        intake.followMotionProfile();

        if (Math.abs(gamepad2.left_stick_y) > 0.2) {
            intake.setPower(-gamepad2.left_stick_y * 0.7);
            timer.reset();
        }
        else{
            intake.setPower(0);
        }


        drive.update();


        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
