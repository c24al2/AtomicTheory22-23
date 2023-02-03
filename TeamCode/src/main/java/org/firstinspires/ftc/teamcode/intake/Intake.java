package org.firstinspires.ftc.teamcode.intake;

import com.acmerobotics.dashboard.FtcDashboard;
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

    public static double INTAKE_POWER_THRESHOLD = 0.02;

    public static double MAX_LIFT_HEIGHT = 1800; // In ticks

    public static double MAX_VEL = 1000;
    public static double MAX_ACCEL = 400;
    public static double MAX_JERK = 0;  // Jerk isn't used if it's 0, but it might end up being necessary

    public ElapsedTime timer;

    public DcMotorEx intake;
    public Servo clawServo;

    public PIDFController controller;
    public MotionProfile motionProfile;

    public Intake(HardwareMap hardwareMap) {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        timer = new ElapsedTime();

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        clawServo.setPosition(0.5);
    }

    public void createMotionProfile(double targetPosition) {
        // Add bounds so that the lift can not go too high or too low
        if (targetPosition < 0) {
            targetPosition = 0;
        } else if (targetPosition > MAX_LIFT_HEIGHT) {
            targetPosition = MAX_LIFT_HEIGHT;
        }

        motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(intake.getCurrentPosition(), intake.getVelocity()),
                new MotionState(targetPosition, 0, 0),
                MAX_VEL,
                MAX_ACCEL,
                MAX_JERK
        );

        timer.reset();
    }

    public void setPower(double power) {
        motionProfile = null;
        intake.setPower(power);
        controller.setTargetPosition(intake.getCurrentPosition());
        controller.setTargetVelocity(0);
        controller.setTargetAcceleration(0);
    }

    public void stepController() {
        if (motionProfile != null) {
            MotionState state = motionProfile.get(timer.time());
            controller.setTargetPosition(state.getX());
            controller.setTargetVelocity(state.getV());
            controller.setTargetAcceleration(state.getA());
        }

        double power = controller.update(intake.getCurrentPosition(), intake.getVelocity());
        intake.setPower(power);
    }
}
