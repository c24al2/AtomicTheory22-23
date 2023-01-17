package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.WHEEL_RADIUS;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.ArrayList;
import java.util.List;

/**
 * Omni drive kinematic equations. All wheel positions and velocities are given starting with front left and
 * proceeding counter-clockwise (i.e., front left, back wheel, front right). Robot poses are specified in a
 * coordinate system with positive x pointing forward, positive y pointing left, and positive heading measured
 * counter-clockwise from the x-axis.
 *
 * [This YouTube video](https://www.youtube.com/watch?v=NcOT9hOsceE) provides a derivation.
 */
public class OmniKinematics {
    /**
     * Computes the wheel velocities corresponding to [robotVel] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotVel           velocity of the robot in its reference frame
     * @param wheelPositions wheel positions *relative* to the robots reference frame
     */
    public static List<Double> robotToWheelVelocities(Pose2d robotVel, List<Pose2d> wheelPositions) {
        List<Double> wheelVelocities = new ArrayList<>(wheelPositions.size());
        for (Pose2d wheelPosition : wheelPositions) {
            double cos = Math.cos(wheelPosition.getHeading());
            double sin = Math.sin(wheelPosition.getHeading());
            double wComponent = robotVel.getHeading() * (wheelPosition.getX()*sin - wheelPosition.getY()*cos);
            double xComponent = robotVel.getX() * cos;
            double yComponent = robotVel.getY() * sin;
            double wheelVelocity = (wComponent + xComponent + yComponent) / WHEEL_RADIUS;
            wheelVelocities.add(wheelVelocity);
        }
        return wheelVelocities;
    }

    /**
     * Computes the wheel accelerations corresponding to [robotAccel] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotAccel acceleration of the robot in its reference frame
     * @param wheelPositions wheel positions *relative* to the robots reference frame
     */
    // follows from linearity of the derivative
    public static List<Double> robotToWheelAccelerations(Pose2d robotAccel, List<Pose2d> wheelPositions) {
        return robotToWheelVelocities(robotAccel, wheelPositions);
    }
}
