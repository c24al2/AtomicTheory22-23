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
public class OdometryLocalizer extends ThreeTrackingWheelLocalizer {
    private final static List<Pose2d> WHEEL_POSES = Arrays.asList(
            new Pose2d(1, 0, Math.toRadians(270)), //back wheel
            new Pose2d(Math.sin(Math.toRadians(30)), Math.cos(Math.toRadians(30)), Math.toRadians(30)), // right
            new Pose2d(Math.sin(Math.toRadians(30)), Math.cos(Math.toRadians(30)), Math.toRadians(150))  // left
    );

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.984; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    private final Encoder frontEncoder;
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

    public OdometryLocalizer(HardwareMap hardwareMap) {
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
                encoderTicksToInches(frontEncoder.getCurrentPosition()),
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()),
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity())
        );
    }
}
