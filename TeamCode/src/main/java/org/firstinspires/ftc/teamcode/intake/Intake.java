package org.firstinspires.ftc.teamcode.intake;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileBuilder;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionSegment;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.List;

// https://github.com/NoahBres/VelocityPIDTuningTutorial/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SampleLinkedPIDUse.java
@Config
public class Intake {
    public static PIDCoefficients INTAKE_PID = new PIDCoefficients(.009, 0, 0.0002);
    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;
    public static double kG = 0;

    public static double MOTION_PROFILE_RECREATION_THRESHOLD = 5; // Only recreate the motion profile if target position changes by this many ticks

    public static double MAX_LIFT_HEIGHT = 1800; // In ticks

//    public static double GRAVITY_ACCEL = 5500; // Constant feedforward acceleration (in ticks/sec/sec) to counteract the lift
    public static double MAX_VEL = 1000;
    public static double MAX_ACCEL = 1000;
    public static double MAX_JERK = 0;  // Jerk isn't used if it's 0, but it might end up being necessary

    public ElapsedTime timer;

    public DcMotorEx intake;
    public Servo clawServo;

    private PIDFController controller;
    public MotionProfile motionProfile;

    public double power = 0;

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

        controller = new PIDFController(INTAKE_PID, kV, kA, kStatic, (x, v) -> kG);
        motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(intake.getCurrentPosition(), intake.getVelocity(), 0),
                new MotionState(intake.getCurrentPosition(), 0, 0),
                MAX_VEL,
                MAX_ACCEL,
                MAX_JERK
        );
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

    public boolean motionProfileHasReachedEnd() {
        return timer.time() >= motionProfile.duration();
    }

    public void setTargetPosition(double targetPosition) {
        // Add bounds so that the lift can not go too high or too low
        if (targetPosition < 0) {
            targetPosition = 0;
        } else if (targetPosition > MAX_LIFT_HEIGHT) {
            targetPosition = MAX_LIFT_HEIGHT;
        }

        if (Math.abs(motionProfile.end().getX() - targetPosition) < MOTION_PROFILE_RECREATION_THRESHOLD) {
            return;
        }

        if (motionProfileHasReachedEnd()) {
            motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                    new MotionState(intake.getCurrentPosition(), intake.getVelocity(), 0),
                    new MotionState(targetPosition, 0, 0),
                    MAX_VEL,
                    MAX_ACCEL,
                    MAX_JERK
            );

            timer.reset();
        } else {
            List<MotionSegment> allMotionSegments = motionProfile.getSegments();
            List<MotionSegment> completedSegments = new ArrayList<>();
            MotionState lastState = motionProfile.end();
            double remainingTime = timer.time();
            for (MotionSegment segment : allMotionSegments) {
                // TODO: Check if last segment should be added or not
                if (remainingTime <= segment.getDt()) {
                    lastState = segment.get(remainingTime);
                    break;
                }
                completedSegments.add(segment);
                remainingTime -= segment.getDt();
            }

            motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                    lastState,
                    new MotionState(targetPosition, 0, 0),
                    MAX_VEL,
                    MAX_ACCEL,
                    MAX_JERK
            );

            if (completedSegments.size() > 0) {
                MotionProfile completedProfile = new MotionProfile(completedSegments);
                motionProfile = completedProfile.plus(motionProfile);
            }
        }
    }

    public void setRelativeTargetPosition(double deltaTargetPosition) {
        double newTargetPosition = getTargetPosition() + deltaTargetPosition;
        setTargetPosition(newTargetPosition);
    }

    public void followMotionProfile() {
        MotionState state = motionProfile.get(timer.time());

        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());

        power = controller.update(intake.getCurrentPosition(), intake.getVelocity());
        intake.setPower(power);
    }
}
