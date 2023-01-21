package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;

import java.util.ArrayList;
import java.util.List;

public class OmniLocalizer implements Localizer {
    private final OmniDrive drive;
    private final List<Pose2d> wheelPoses;
    private final boolean useExternalHeading;

    private Pose2d _poseEstimate;
    private Pose2d poseVelocity;
    private List<Double> lastWheelPositions;
    private double lastExtHeading;

    public OmniLocalizer(OmniDrive drive, List<Pose2d> wheelPoses, boolean useExternalHeading) {
        this.drive = drive;
        this.wheelPoses = wheelPoses;
        this.useExternalHeading = useExternalHeading;

        this._poseEstimate = new Pose2d();
        this.poseVelocity = null;
        this.lastWheelPositions = new ArrayList<>();
        this.lastExtHeading = Double.NaN;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return this._poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d value) {
        lastWheelPositions = new ArrayList<>();
        lastExtHeading = Double.NaN;
        if (useExternalHeading) {
            drive.setExternalHeading(value.getHeading());
        }
        _poseEstimate = value;
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
            Pose2d robotPoseDelta = OmniKinematics.wheelToRobotVelocities(wheelDeltas, wheelPoses);
            double finalHeadingDelta = useExternalHeading ? Angle.normDelta(extHeading - lastExtHeading) : robotPoseDelta.getHeading();
            _poseEstimate = Kinematics.relativeOdometryUpdate(_poseEstimate, new Pose2d(robotPoseDelta.vec(), finalHeadingDelta));
        }

        List<Double> wheelVelocities = drive.getWheelVelocities();
        Double extHeadingVel = drive.getExternalHeadingVelocity(); // Use Double class instead of double primitive so that it can be null
        if (wheelVelocities != null) {
            poseVelocity = OmniKinematics.wheelToRobotVelocities(wheelVelocities, wheelPoses);
            if (useExternalHeading && extHeadingVel != null) {
                poseVelocity = new Pose2d(poseVelocity.vec(), extHeadingVel);
            }
        }

        lastWheelPositions = wheelPositions;
        lastExtHeading = extHeading;
    }
}