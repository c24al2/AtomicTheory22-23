package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class IntakeandLiftPID{
    public ElapsedTime timer = new ElapsedTime();
    public static PIDFCoefficients coeffs = new PIDFCoefficients(.009, 0, 0.0002, 0);
    public double currentVelocity = 0;
    public double targetVelocity = 0;
    public double velocityError = 0;
    public double p = coeffs.p;
    public double i = coeffs.i;
    public double d = coeffs.d;
    public double f = coeffs.f;
    double lastP;
    double lastI;
    double lastD;
    double lastF;
    boolean onEncoders = true;

    public DcMotorEx intake;
    public Servo clawServo;
    public static double maxVelocity = 133000;
    public static double maxAcceleration = 2000;
    // Jerk isn't used if it's 0, but it might end up being necessary
    public static double maxJerk = 0;
    float targetPosition = 0;

    MotionProfile profile;

    public IntakeandLiftPID(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        coeffs = intake.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        p = coeffs.p;
        i = coeffs.i;
        d = coeffs.d;
        f = coeffs.f;
        lastP = p;
        lastI = i;
        lastD = d;
        lastF = f;
    }

    public void clawOpen(){
        clawServo.setPosition(0);
    }
    public void clawClose(){
        clawServo.setPosition(0.5);
    }

    public MotionProfile generateProfile(int targetTicks){
        MotionProfile newProfile = generateMotionProfile(targetTicks);
        return newProfile;
    }
    public boolean intakeLift(MotionProfile profile1) {
        return followMotionProfile(profile1);
    }

    public void intakeFullStep(int targetTicks){
        MotionProfile activeProfile = generateProfile(targetTicks);
        intakeLift(activeProfile);
    }


    public void intakeLiftEasy(int targetTicks) {
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setTargetPosition(targetTicks);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setPower(0.7);
    }

    MotionProfile generateMotionProfile(double ticks) {
        if (ticks == 0){
            return null;
        }

        // Based on 60RPM motor, adjust if different
        return MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(intake.getCurrentPosition(), 1600, 0),
                new MotionState(ticks, 0, 0),
                maxVelocity,
                maxAcceleration,
                maxJerk);
    }

    boolean followMotionProfile(MotionProfile profile){
        // specify coefficients/gains
        // create the controller
        MotionState state = profile.get(timer.time());
        if (profile.end().getX() < state.getX() && state.getX() < profile.start().getX() || profile.start().getX() < state.getX() && state.getX() < profile.end().getX()) {
            currentVelocity = intake.getVelocity();
            targetVelocity = state.getV();
            velocityError = state.getV() - intake.getVelocity();
            // in each iteration of the control loop
            // measure the position or output variable
            // apply the correction to the input variable
            intake.setVelocity(state.getV());
            return false;
        } else {
            intake.setPower(0);
            timer.reset();
            this.profile = generateMotionProfile(intake.getCurrentPosition());
            return true;
        }
    }

    public void run(Gamepad gamepad) {
        if (gamepad.x) {
            // Ability for manual control, which resets the motor's encoder value when done
            if (onEncoders) {
                onEncoders = false;
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            intake.setPower(-gamepad.left_stick_y * 0.7);
        } else {
            if (!onEncoders) {
                // Resetting the encoder value
                intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                targetPosition = 0;
                onEncoders = true;
            }
            targetPosition -= gamepad.left_stick_y * 80;
            targetPosition = Range.clip(targetPosition, 0, 4000);
            intakeLiftEasy((int) targetPosition);
        }
    }
}