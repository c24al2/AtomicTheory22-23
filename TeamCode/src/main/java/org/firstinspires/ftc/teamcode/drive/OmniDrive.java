package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.localization.Localizer;

import java.util.List;

public abstract class OmniDrive extends Drive {
    private final double kV;
    private final double kA;
    private final double kStatic;
    private final List<Pose2d> wheelPoses;
    private Localizer localizer;

    public OmniDrive(double kV, double kA, double kStatic, List<Pose2d> wheelPoses, boolean useExternalHeading) {
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.wheelPoses = wheelPoses;
        this.localizer = new OmniLocalizer(this, wheelPoses, useExternalHeading);
    }

    @NonNull
    @Override
    public Localizer getLocalizer() {
        return this.localizer;
    }

    @Override
    public void setLocalizer(@NonNull Localizer localizer) {
        this.localizer = localizer;
    }

    @Override
    public void setDriveSignal(@NonNull DriveSignal driveSignal) {
        List<Double> velocities = OmniKinematics.robotToWheelVelocities(driveSignal.getVel(), wheelPoses);
        List<Double> accelerations = OmniKinematics.robotToWheelAccelerations(driveSignal.getAccel(), wheelPoses);
        List<Double> powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic);
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2));
    }

    // TODO: Look at this function
    @Override
    public void setDrivePower(@NonNull Pose2d drivePower) {
        List<Double> powers = OmniKinematics.robotToWheelVelocities(drivePower, wheelPoses);
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2));
    }

    /**
     * Sets the following motor powers (normalized voltages). All arguments are on the interval `[-1.0, 1.0]`.
     */
    abstract void setMotorPowers(double leftMotor, double backMotor, double rightMotor);

    /**
     * Returns the positions of the wheels in linear distance units. Positions should exactly match the ordering in
     * [setMotorPowers].
     */
    abstract List<Double> getWheelPositions();

    /**
     * Returns the velocities of the wheels in linear distance units. Positions should exactly match the ordering in
     * [setMotorPowers].
     */
    List<Double> getWheelVelocities() {
        return null;
    }
}
