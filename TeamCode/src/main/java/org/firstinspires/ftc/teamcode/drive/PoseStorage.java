package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseStorage {
    // This static keyword lets us share the data between opmodes.
    public static Pose2d currentPose = new Pose2d();
}