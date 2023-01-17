package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;

import java.util.ArrayList;
import java.util.List;

public abstract class OmniDrive extends Drive {

    static class OmniLocalizer implements Localizer {
        private OmniDrive drive;
        private boolean useExternalHeading;
        private Pose2d poseEstimate;
        private Pose2d poseVelocity;
        private List<Double> lastWheelPositions;
        private double lastExtHeading;

        /**
         * Default localizer for omni drives based on the drive encoders and (optionally) a heading sensor.
         *
         * @param drive drive
         * @param useExternalHeading use external heading provided by an external sensor (e.g., IMU, gyroscope)
         */
        public OmniLocalizer(OmniDrive drive, boolean useExternalHeading) {
            this.drive = drive;
            this.useExternalHeading = useExternalHeading;
            this.poseEstimate = new Pose2d();
            this.poseVelocity = null;
            this.lastWheelPositions = new ArrayList<>();
            this.lastExtHeading = Double.NaN;
        }

        @NonNull
        @Override
        public Pose2d getPoseEstimate() {
            return poseEstimate;
        }

        @Override
        public void setPoseEstimate(@NonNull Pose2d poseEstimate) {
            this.lastWheelPositions = new ArrayList<>();
            this.lastExtHeading = Double.NaN;
            if (useExternalHeading) {
                drive.setExternalHeading(poseEstimate.getHeading());
            }
            this.poseEstimate = poseEstimate;
        }

        @Nullable
        @Override
        public Pose2d getPoseVelocity() {
            return this.poseVelocity;
        }

        @Override
        public void update() {
            List<Double> wheelPositions = drive.getWheelPositions();
            double extHeading = useExternalHeading ? drive.getExternalHeading() : Double.NaN;
            if (!lastWheelPositions.isEmpty()) {
                List<Double> wheelDeltas = new ArrayList<>();
                for (int i = 0; i < wheelPositions.size(); i++) {
                    wheelDeltas.add(wheelPositions.get(i) - lastWheelPositions.get(i));
                }

                Pose2d robotPoseDelta = OmniKinematics.wheelToRobotVelocities(wheelDeltas, drive.wheelPositions);

                double finalHeadingDelta = useExternalHeading ? Angle.normDelta(extHeading - lastExtHeading) : robotPoseDelta.getHeading();

                poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, new Pose2d(robotPoseDelta.vec(), finalHeadingDelta));
            }

            List<Double> wheelVelocities = drive.getWheelVelocities();
            Double extHeadingVel = drive.getExternalHeadingVelocity(); // Using Double instead of double to allow it to be null
            if (wheelVelocities != null) {
                poseVelocity = OmniKinematics.wheelToRobotVelocities(wheelVelocities, drive.wheelPositions);
                if (useExternalHeading && extHeadingVel != null) {
                    poseVelocity = new Pose2d(poseVelocity.vec(), extHeadingVel);
                }
            }

            lastWheelPositions = wheelPositions;
            lastExtHeading = extHeading;
        }
    }

    private final double kV;
    private final double kA;
    private final double kStatic;
    private final List<Pose2d> wheelPositions;
    private Localizer localizer;

    public OmniDrive(double kV, double kA, double kStatic, List<Pose2d> wheelPositions, boolean useExternalHeading) {
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.wheelPositions = wheelPositions;
        this.localizer = new OmniLocalizer(this, useExternalHeading);
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
    protected double getRawExternalHeading() {
        return 0;
    }

    @Override
    public void setDrivePower(@NonNull Pose2d drivePower) {
        List<Double> powers = OmniKinematics.robotToWheelVelocities(drivePower, wheelPositions);
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2));
    }

    @Override
    public void setDriveSignal(@NonNull DriveSignal driveSignal) {
        List<Double> velocities = OmniKinematics.robotToWheelVelocities(driveSignal.getVel(), wheelPositions);
        List<Double> accelerations = OmniKinematics.robotToWheelAccelerations(driveSignal.getAccel(), wheelPositions);
        List<Double> powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic);
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
