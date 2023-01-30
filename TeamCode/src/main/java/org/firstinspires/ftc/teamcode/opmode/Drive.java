package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.IntakeandLiftPID;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleOmniDrive;


@Config
@TeleOp
public class Drive extends OpMode {
    private static double SLOW_MODE_SCALAR = 0.4;

    // TODO: Remove, this is for debugging purposes
//        private static Pose2d START_POSE = PoseStorage.currentPose;
    private static Pose2d START_POSE = new Pose2d(36, -62.8, Math.toRadians(90));

    private boolean driverSlowMode = false;
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();

    public SampleOmniDrive drive;
    public IntakeandLiftPID liftandServo;

    @Override
    public void init() {
        drive = new SampleOmniDrive(hardwareMap);
        liftandServo = new IntakeandLiftPID(hardwareMap);

        drive.setPoseEstimate(START_POSE);
    }

    @Override
    public void loop() {
        // Enable toggling driver slow mode when press a
        if (!previousGamepad1.a && gamepad1.a) {
            driverSlowMode = !driverSlowMode;
        }

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        Vector2d translationalInput = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
        // Then, rotate that vector by the inverse of the robots' DELTA heading
        // AKA driver lock. Disabling this line will disable driver lock
        translationalInput = translationalInput.rotated(-(poseEstimate.getHeading() - START_POSE.getHeading()));

        Pose2d input = new Pose2d(translationalInput, -gamepad1.right_stick_x);
        if (driverSlowMode) {
            input = input.times(SLOW_MODE_SCALAR);
        }

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(input);

//        if (Math.abs(gamepad2.left_stick_y) > 0.2) {
//            liftandServo.setIntakePower(-gamepad2.left_stick_y);
//        }

        if (gamepad2.x){
            liftandServo.clawClose();
        }
        if (gamepad2.y){
            liftandServo.clawOpen();
        }

        if (!previousGamepad2.dpad_up && gamepad2.dpad_up) {
            liftandServo.generateMotionProfile(liftandServo.distanceToEncoders(IntakeandLiftPID.HIGHJUNCTION));
        }

        if (!previousGamepad2.dpad_down && gamepad2.dpad_down) {
            liftandServo.generateMotionProfile(liftandServo.distanceToEncoders(IntakeandLiftPID.GROUNDJUNCTION));
        }

        if (!previousGamepad2.dpad_left && gamepad2.dpad_left) {
            liftandServo.generateMotionProfile(liftandServo.distanceToEncoders(IntakeandLiftPID.MEDIUMJUNCTION));

        }
        if (!previousGamepad2.dpad_right && gamepad2.dpad_right) {
            liftandServo.generateMotionProfile(liftandServo.distanceToEncoders(IntakeandLiftPID.LOWJUNCTION));
        }

        telemetry.addData("Current Position", liftandServo.currentPosition);
        telemetry.addData("Current Velocity", liftandServo.currentVelocity);
        telemetry.addData("Target Position", liftandServo.targetPosition);
        telemetry.addData("Target Velocity", liftandServo.targetVelocity);

        drive.update();
        liftandServo.followMotionProfile();

        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
