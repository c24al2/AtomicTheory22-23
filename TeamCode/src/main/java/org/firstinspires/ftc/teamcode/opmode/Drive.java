package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.IntakeandLiftPID;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleOmniDrive;

@TeleOp
public class Drive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleOmniDrive drive = new SampleOmniDrive(hardwareMap);
        IntakeandLiftPID liftandServo = new IntakeandLiftPID(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        drive.setPoseEstimate(PoseStorage.currentPose);
        drive.setPoseEstimate(new Pose2d(36, -62.8, Math.toRadians(90)));

        waitForStart();

        while (!isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );//.rotated(-poseEstimate.getHeading());

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

            liftandServo.run(gamepad2);

            if (gamepad2.x){
                liftandServo.clawClose();
            }
            if (gamepad2.y){
                liftandServo.clawOpen();
            }
//            if (gamepad2.dpad_up){
//                liftandServo.intakeFullStep(liftandServo.distanceToEncoders(DriveConstants.HIGHJUNCTION));
//            }
//            if (gamepad2.dpad_down){
//                liftandServo.intakeFullStep(liftandServo.distanceToEncoders(DriveConstants.GROUNDJUNCTION));
//            }
//            if (gamepad2.dpad_left){
//                liftandServo.intakeFullStep(liftandServo.distanceToEncoders(DriveConstants.MEDIUMJUNCTION));
//            }
//            if (gamepad2.dpad_right){
//                liftandServo.intakeFullStep(liftandServo.distanceToEncoders(DriveConstants.LOWJUNCTION));
//            }
        }


    }
}
