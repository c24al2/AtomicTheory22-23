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
    private static RealMatrix calculateHMatrix(List<Pose2d> wheelPositions) {
        RealMatrix hMatrix = MatrixUtils.createRealMatrix(3, 3);
        for (int i = 0; i < wheelPositions.size(); i++) {
            Pose2d wheelPosition = wheelPositions.get(i);
            double cos = Math.cos(wheelPosition.getHeading());
            double sin = Math.sin(wheelPosition.getHeading());
            double[] rowData = {wheelPosition.getX() * sin - wheelPosition.getY() * cos, cos, sin};
            hMatrix.setRow(i, rowData);
        }
        hMatrix = hMatrix.scalarMultiply((1 / WHEEL_RADIUS));
        return hMatrix;
    }

    /**
     * Computes the wheel velocities corresponding to [robotVel] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotVel       velocity of the robot in its reference frame
     * @param wheelPositions wheel positions *relative* to the robots reference frame
     */
    public static List<Double> robotToWheelVelocities(Pose2d robotVel, List<Pose2d> wheelPositions) {
        RealMatrix hMatrix = calculateHMatrix(wheelPositions);
        double[] robotVelMatrixData = {robotVel.getHeading(), robotVel.getX(), robotVel.getY()};
        RealMatrix robotVelMatrix = MatrixUtils.createColumnRealMatrix(robotVelMatrixData);
        RealMatrix wheelVelocitiesMatrix = hMatrix.multiply(robotVelMatrix);

        List<Double> wheelVelocities = new ArrayList<>(wheelPositions.size());
        for (double wheelVelocity : wheelVelocitiesMatrix.getColumn(0)) {
            wheelVelocities.add(wheelVelocity);
        }
        return wheelVelocities;
    }

    /**
     * Computes the wheel accelerations corresponding to [robotAccel] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotAccel     acceleration of the robot in its reference frame
     * @param wheelPositions wheel positions *relative* to the robots reference frame
     */
    // follows from linearity of the derivative
    public static List<Double> robotToWheelAccelerations(Pose2d robotAccel, List<Pose2d> wheelPositions) {
        return robotToWheelVelocities(robotAccel, wheelPositions);
    }

    /**
     * Computes the robot velocity corresponding to [wheelVelocities] and the given drive parameters.
     *
     * @param wheelVelocities    wheel velocities (or wheel position deltas)
     * @param omniWheelPositions wheel positions *relative* to the robots reference frame
     */
    public static Pose2d wheelToRobotVelocities(List<Double> wheelVelocities, List<Pose2d> omniWheelPositions) {
        double[] wheelVelocitiesMatrixData = new double[wheelVelocities.size()];
        for (int i = 0; i < wheelVelocities.size(); i++) {
            wheelVelocitiesMatrixData[i] = wheelVelocities.get(i);
        }
        RealMatrix wheelVelocitiesMatrix = MatrixUtils.createColumnRealMatrix(wheelVelocitiesMatrixData);

        RealMatrix hMatrix = calculateHMatrix(omniWheelPositions);
        RealMatrix inverseHMatrix = MatrixUtils.inverse(hMatrix);

        RealMatrix robotVelMatrix = inverseHMatrix.multiply(wheelVelocitiesMatrix);

        return new Pose2d(
                robotVelMatrix.getEntry(1, 0),
                robotVelMatrix.getEntry(2, 0),
                robotVelMatrix.getEntry(0, 0)
        );
    }
}