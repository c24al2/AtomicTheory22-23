package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class IntakeandLiftPID {
    public static double TICKS_PER_REV = 384.5;
    public static double SPOOL_RADIUS = .75;

    public static PIDFCoefficients INTAKE_PID = new PIDFCoefficients(.009, 0, 0.0002, 0);

    public static double MAX_VEL = 62000;
    public static double MAX_ACCEL = 1000;
    public static double MAX_JERK = 0;  // Jerk isn't used if it's 0, but it might end up being necessary
    public static double POWER_WEIGHT = 0.8;

    //Junction Positions listed in inches, later converted to encoder ticks
    public static double HIGHJUNCTION = 4.5;
    public static double MEDIUMJUNCTION = 3.45;
    public static double LOWJUNCTION = 2.2;
    public static double GROUNDJUNCTION = 0.563;
    public static double PICKUP_CONE_1 = 0.65;
    public static double PICKUP_CONE_2 = 0.55;
    public static double PICKUP_CONE_3 = 0.45;
    public static double PICKUP_CONE_4 = 0.35;
    public static double PICKUP_CONE_5 = 0.25;

    // TODO: Make private when we don't need them to be public anymore
    public ElapsedTime timer;
    // TODO: Remove telemetry variables
    public double currentVelocity = 0;
    public double targetVelocity = 0;
    public double velocityError = 0;

    public DcMotorEx intake;
    public Servo clawServo;

    private MotionProfile storedProfile;

    public IntakeandLiftPID(HardwareMap hardwareMap) {
        timer = new ElapsedTime();
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);
//        intake.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, INTAKE_PID);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void clawOpen(){
        clawServo.setPosition(1.0);
    }
    public void clawClose(){
        clawServo.setPosition(0.8);
    }

    public int distanceToEncoders(double distance){
        double encoderRatio = TICKS_PER_REV/2 * Math.PI * SPOOL_RADIUS;
        double encoderConverted = distance*encoderRatio;
        int intEncoder = (int) encoderConverted;
        return intEncoder;
    }

    public void generateMotionProfile(double ticks) {
        storedProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(intake.getCurrentPosition(), intake.getVelocity(), 0),
                new MotionState(ticks, 0, 0),
                MAX_VEL,
                MAX_ACCEL,
                MAX_JERK
        );

        timer.reset();
    }

    public void followMotionProfile() {
        if (storedProfile == null) return;

        MotionState state = storedProfile.get(timer.time());

        currentVelocity = intake.getVelocity();
        targetVelocity = state.getV();
        velocityError = targetVelocity - currentVelocity;

        intake.setTargetPosition((int) state.getX());
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setVelocity(state.getV());
    }

    public void setIntakePower(double power) {
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(power * POWER_WEIGHT);
    }
}
