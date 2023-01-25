package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
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
public class IntakeandLiftPID {
    public ElapsedTime timer;
    public static PIDFCoefficients coeffs = new PIDFCoefficients(.009, 0, 0.0002, 0);
    public double currentVelocity = 0;
    public double targetVelocity = 0;
    public double velocityError = 0;

    public DcMotorEx intake;
    public Servo clawServo;
    public static double maxVelocity = 133000;
    public static double maxAcceleration = 2000;
    public static double maxJerk = 0;  // Jerk isn't used if it's 0, but it might end up being necessary

    public MotionProfile storedProfile;

    public IntakeandLiftPID(HardwareMap hardwareMap) {
        timer = new ElapsedTime();
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
    }

    public void clawOpen(){
        clawServo.setPosition(-1.0);
    }
    public void clawClose(){
        clawServo.setPosition(1.0);
    }

    public MotionProfile generateProfile(int targetTicks){
        MotionProfile newProfile = generateMotionProfile(targetTicks);
        return newProfile;
    }
    public boolean intakeLift(MotionProfile profile1) {
        return followMotionProfile(profile1);
    }

    public MotionProfile intakeFullStep(int targetTicks){
        MotionProfile activeProfile = generateProfile(targetTicks);
        intakeLift(activeProfile);
        storedProfile = activeProfile;
        return storedProfile;
    }

    public void update(){
        MotionState state = storedProfile.get(timer.time());
        if (storedProfile.end().getX() < state.getX() && state.getX() < storedProfile.start().getX() || storedProfile.start().getX() < state.getX() && state.getX() < storedProfile.end().getX()){
            followMotionProfile(storedProfile);
        }
    }


    public void intakeLiftEasy(int targetTicks) {
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setTargetPosition(targetTicks);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setPower(0.7);
    }

    public int distanceToEncoders(double distance){
        double encoderRatio = DriveConstants.LIFT_ENCODER_RES/2 * Math.PI * DriveConstants.SPOOL_RADIUS;
        double encoderConverted = distance*encoderRatio;
        int intEncoder = (int) encoderConverted;
        return intEncoder;
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

    boolean followMotionProfile(MotionProfile profile2){
        // specify coefficients/gains
        // create the controller
        MotionState state = profile2.get(timer.time());
        if (profile2.end().getX() < state.getX() && state.getX() < profile2.start().getX() || profile2.start().getX() < state.getX() && state.getX() < profile2.end().getX()) {
            currentVelocity = intake.getVelocity();
            targetVelocity = state.getV();
            velocityError = targetVelocity - currentVelocity;
            // in each iteration of the control loop
            // measure the position or output variable
            // apply the correction to the input variable
            intake.setVelocity(state.getV());
            return false;
        } else {
            intake.setPower(0);
            timer.reset();
            this.storedProfile = generateMotionProfile(intake.getCurrentPosition());
            return true;
        }
    }

    public void run(Gamepad gamepad) {
            // Ability for manual control, which resets the motor's encoder value when done
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setPower(-gamepad.left_stick_y * 0.7);
        }
    }
