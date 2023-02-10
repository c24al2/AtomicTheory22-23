package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

@Config
public class ThreeWheelOdometryLocalizer extends ThreeTrackingWheelLocalizer {
    private final static List<Pose2d> WHEEL_POSES = Arrays.asList(
            new Pose2d(1.7, 0.22, Math.toRadians(180)), //front wheel
            new Pose2d(-1.161, 1.79, Math.toRadians(120)),  // "left" wheel
            new Pose2d(-1.161, -1.79, Math.toRadians(420)) // "right" wheel
    );

    public static double TICKS_PER_REV = 800;
    public static double WHEEL_RADIUS = 0.984; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double FRONT_ENCODER_MULTIPLIER = 1;
    public static double LEFT_ENCODER_MULTIPLIER = 1;
    public static double RIGHT_ENCODER_MULTIPLIER = 1;

    private final Encoder frontEncoder;
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

    public ThreeWheelOdometryLocalizer(HardwareMap hardwareMap) {
        super(WHEEL_POSES);

        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "bm"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rm"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lm"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * FRONT_ENCODER_MULTIPLIER,
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * LEFT_ENCODER_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * RIGHT_ENCODER_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(frontEncoder.getRawVelocity()) * FRONT_ENCODER_MULTIPLIER,
                encoderTicksToInches(leftEncoder.getRawVelocity()) * LEFT_ENCODER_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getRawVelocity()) * RIGHT_ENCODER_MULTIPLIER
        );
    }
}
