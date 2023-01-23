package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static Pose2d START_POSE = new Pose2d(-60, -35, Math.toRadians(0));
    public static Pose2d PLACE_PRELOADED_CONE_POSE = new Pose2d(-7, -20, Math.toRadians(-35.5));
    public static Pose2d PICKUP_CONE_FROM_STACK_POSE = new Pose2d(-12, -63, Math.toRadians(-90));
    public static Pose2d PLACE_STACK_CONE_POSE = new Pose2d(-7, -28, Math.toRadians(35.5));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(42.43, 42.43, Math.toRadians(367.9), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(PLACE_STACK_CONE_POSE)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-32, -36, Math.toRadians(180)), Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}