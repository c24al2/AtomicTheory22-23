package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.IntakeandLiftPID;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleOmniDrive;

@TeleOp
public class Drive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleOmniDrive drive = new SampleOmniDrive(hardwareMap);
        IntakeandLiftPID liftandServo = new IntakeandLiftPID(hardwareMap);

        // TODO: Remove, this is for debugging purposes
//        Pose2d startPose = PoseStorage.currentPose;
        Pose2d startPose = new Pose2d(36, -62.8, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        waitForStart();

        while (!isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
            // Then, rotate that vector by the inverse of the robots' DELTA heading
            input = input.rotated(-(poseEstimate.getHeading() - startPose.getHeading()));

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            liftandServo.setIntakePower(-gamepad2.left_stick_y);

            if (gamepad2.x){
                liftandServo.clawClose();
            }
            if (gamepad2.y){
                liftandServo.clawOpen();
            }
            // TODO: Add this code back later (it's more complex than these lines)
            // Calling followMotionProfile will only do one step along the motion profile
            // Basically as it's written it won't raise the lift all the way
            if (gamepad2.dpad_up) {
                liftandServo.generateMotionProfile(liftandServo.distanceToEncoders(IntakeandLiftPID.HIGHJUNCTION));
                while (true) {
                    liftandServo.followMotionProfile();
                }
            }
            if (gamepad2.dpad_down) {
                liftandServo.generateMotionProfile(liftandServo.distanceToEncoders(IntakeandLiftPID.GROUNDJUNCTION));
                while (true) {
                    liftandServo.followMotionProfile();
                }
            }
            if (gamepad2.dpad_left) {
                liftandServo.generateMotionProfile(liftandServo.distanceToEncoders(IntakeandLiftPID.MEDIUMJUNCTION));
                while (true) {
                    liftandServo.followMotionProfile();
                }
            }
            if (gamepad2.dpad_right) {
                liftandServo.generateMotionProfile(liftandServo.distanceToEncoders(IntakeandLiftPID.LOWJUNCTION));
                while (true) {
                    liftandServo.followMotionProfile();
                }
            }
        }


    }
}
