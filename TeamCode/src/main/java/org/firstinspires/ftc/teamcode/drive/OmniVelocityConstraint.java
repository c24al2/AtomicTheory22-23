package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.UnsatisfiableConstraint;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class OmniVelocityConstraint implements TrajectoryVelocityConstraint {
    private final double maxWheelVel;
    private final List<Pose2d> wheelPositions;


    public OmniVelocityConstraint(double maxWheelVel, List<Pose2d> wheelPositions) {
        this.maxWheelVel = maxWheelVel;
        this.wheelPositions = wheelPositions;
    }

    /**
     * Returns the maximum profile velocity.
     *
     * @param s path displacement
     * @param pose pose
     * @param deriv pose derivative
     * @param baseRobotVel additive base velocity
     */
    @Override
    public double get(double s, @NonNull Pose2d pose, @NonNull Pose2d deriv, @NonNull Pose2d baseRobotVel) {
        List<Double> baseWheelVelocities = OmniKinematics.robotToWheelVelocities(baseRobotVel, wheelPositions);

        List<Double> destination = new ArrayList<>(wheelPositions.size());

        for (double wheelVelocity : baseWheelVelocities) {
            destination.add(Math.abs(wheelVelocity));
        }

        double largestWheelVelocity = Collections.max(destination);

        if (largestWheelVelocity >= this.maxWheelVel) {
            throw new UnsatisfiableConstraint();
        } else {
            Pose2d robotDeriv = Kinematics.fieldToRobotVelocity(pose, deriv);
            List<Double> derivWheelVelocities = OmniKinematics.robotToWheelVelocities(robotDeriv, wheelPositions);

            List<Double> destination2 = new ArrayList<>(wheelPositions.size());

            for (int i = 0; i < wheelPositions.size(); i++) {
                double firstNum = baseWheelVelocities.get(i);
                double secondNum = derivWheelVelocities.get(i);

                double value1 = (this.maxWheelVel - firstNum) / secondNum;
                double value2 = (-this.maxWheelVel - firstNum) / secondNum;

                destination2.add(Math.max(value1, value2));
            }

            return Collections.min(destination2);
        }
    }
}
