package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static Pose2d START_POSE = new Pose2d(36, -62.8, Math.toRadians(90));
    public static Pose2d PLACE_PRELOADED_CONE_POSE = new Pose2d(21, -4, Math.toRadians(55));
    public static Pose2d PICKUP_CONE_FROM_STACK_POSE = new Pose2d(60, -12, Math.toRadians(0));
    public static Pose2d PLACE_STACK_CONE_POSE = new Pose2d(31, -7, Math.toRadians(135));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(270), Math.toRadians(90), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(PLACE_PRELOADED_CONE_POSE)
                                .setReversed(true)
                                .splineToLinearHeading(PICKUP_CONE_FROM_STACK_POSE, PICKUP_CONE_FROM_STACK_POSE.getHeading())
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}