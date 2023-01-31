package org.firstinspires.ftc.teamcode.intake;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

// https://github.com/NoahBres/VelocityPIDTuningTutorial/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SampleLinkedPIDUse.java
@Config
public class Intake {
    public static PIDCoefficients INTAKE_PID = new PIDCoefficients(.009, 0, 0.0002);
    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;

    public static double MAX_LIFT_HEIGHT = 2800; // In ticks

    public static double GRAVITY_ACCEL = 200; // Constant feedforward acceleration (in ticks/sec/sec) to counteract the lift
    public static double MAX_VEL = 62000;
    public static double MAX_ACCEL = 2000;
    public static double MAX_JERK = 0;  // Jerk isn't used if it's 0, but it might end up being necessary

    public ElapsedTime timer;

    public DcMotorEx intake;
    public Servo clawServo;

    private PIDFController controller;
    private MotionProfile motionProfile;

    public Intake(HardwareMap hardwareMap) {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        timer = new ElapsedTime();

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        controller = new PIDFController(INTAKE_PID, kV, kA, kStatic);
    }

    public void openClaw() {
        clawServo.setPosition(1.0);
    }
    public void closeClaw(){
        clawServo.setPosition(0.8);
    }

    public double getTargetPosition() {
        return motionProfile.end().getX();
    }

    public void setTargetPosition(double targetPosition) {
        // Add bounds so that the lift can not go too high or too low
        if (targetPosition < 0) {
            targetPosition = 0;
        } else if (targetPosition > MAX_LIFT_HEIGHT) {
            targetPosition = MAX_LIFT_HEIGHT;
        }

        motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(intake.getCurrentPosition(), intake.getVelocity(), 0),
                new MotionState(targetPosition, 0, 0),
                MAX_VEL,
                MAX_ACCEL,
                MAX_JERK
        );

        timer.reset();
    }

    public void setRelativeTargetPosition(double deltaTargetPosition) {
        double newTargetPosition = getTargetPosition() + deltaTargetPosition;
        setTargetPosition(newTargetPosition);
    }

    public void followMotionProfile() {
        MotionState state = motionProfile.get(timer.time());

        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA() + GRAVITY_ACCEL);

        double power = controller.update(intake.getCurrentPosition(), intake.getVelocity());
        intake.setPower(power);
    }
}
