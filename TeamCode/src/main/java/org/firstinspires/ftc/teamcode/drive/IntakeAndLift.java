package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.LIFT_SPEED;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


public class IntakeAndLift {
    public DcMotorEx intake;
    public Servo clawServo;
    float targetPosition = 0;
    boolean onEncoders = true;

    public void init (HardwareMap hardwareMap) {
        final VoltageSensor batteryVoltageSensor;
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        final DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        final Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void run(Gamepad gamepad) {
        if (gamepad.x) {
            // Ability for manual control, which resets the motor's encoder value when done
            if (onEncoders) {
                onEncoders = false;
                intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            }
                intake.setPower(-gamepad.left_stick_y * 0.7);
        } else {
            if (!onEncoders) {
                // Resetting the encoder value
                intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                targetPosition = 0;
                onEncoders = true;
            }
            targetPosition -= gamepad.left_stick_y * 80;
            targetPosition = Range.clip(targetPosition, 0, 1450);
            goTo((int) targetPosition, LIFT_SPEED);
        }
    }

    public void goTo(int position, double power) {
        intake.setTargetPosition(position);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setPower(power);
    }

    public void clawOpen(){
        clawServo.setPosition(0);
    }
    public void clawClose(){
        clawServo.setPosition(0.5);
    }

    PIDCoefficients coeffs = new PIDCoefficients(0, 0, 0);

    // TODO: Tune PID Coefficients for the goToWithPID fxn
    public void goToWithPID(int position, double velocity){
        while (position - intake.getCurrentPosition() > 10) {
            intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            PIDFController intakeController = new PIDFController(coeffs, kV, kA, kStatic);
            intakeController.setTargetPosition(position);
            intakeController.setTargetVelocity(velocity);
            double measuredPosition = intake.getCurrentPosition();
            double correction = intakeController.update(measuredPosition);
            int correcting = (int)(correction);
            intake.setTargetPosition(position + correcting);
            intake.setVelocity(velocity + correcting);
        }
    }
}
