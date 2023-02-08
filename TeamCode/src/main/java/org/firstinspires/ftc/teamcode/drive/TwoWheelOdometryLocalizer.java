package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

@Config
public class TwoWheelOdometryLocalizer extends TwoTrackingWheelLocalizer {
    private final static List<Pose2d> WHEEL_POSES = Arrays.asList(
            new Pose2d(1.7, 0.22, Math.toRadians(90)), //front wheel
            new Pose2d(-1.161, -1.79, Math.toRadians(210)) // "right" wheel
    );

    public static double TICKS_PER_REV = 200;
    public static double WHEEL_RADIUS = 0.984; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public Encoder frontEncoder;
    public Encoder rightEncoder;

    private final SampleOmniDrive drive;

    public TwoWheelOdometryLocalizer(HardwareMap hardwareMap, SampleOmniDrive drive) {
        super(WHEEL_POSES);

        this.drive = drive;

        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "bm"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lm"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }


    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(frontEncoder.getCurrentPosition()),
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
                encoderTicksToInches(frontEncoder.getRawVelocity()),
                encoderTicksToInches(rightEncoder.getRawVelocity())
        );
    }
}
