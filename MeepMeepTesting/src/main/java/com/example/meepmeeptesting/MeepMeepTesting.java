package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static Pose2d START_POSE = new Pose2d(-62.8, -35, Math.toRadians(0));
    public static Pose2d PLACE_PRELOADED_CONE_POSE = new Pose2d(-1, -17, Math.toRadians(-46));
    public static Pose2d PICKUP_CONE_FROM_STACK_POSE = new Pose2d(-8, -64, Math.toRadians(-90));
    public static Pose2d PLACE_STACK_CONE_POSE = new Pose2d(-1, -31, Math.toRadians(46));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(42.43, 42.43, Math.toRadians(367.9), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(PICKUP_CONE_FROM_STACK_POSE)
//                .addTemporalMarker(() -> liftandServo.clawClose())
//                .addTemporalMarker(() -> liftandServo.intakeFullStep(DriveConstants.MEDIUMJUNCTION))
                                        .waitSeconds(0.1)
                                        .setReversed(true)
                                        .splineToSplineHeading(PLACE_STACK_CONE_POSE, PLACE_STACK_CONE_POSE.getHeading())
//                .addTemporalMarker(() -> liftandServo.intakeFullStep(DriveConstants.HIGHJUNCTION))
//                .addTemporalMarker(() -> liftandServo.clawOpen())
                                        .waitSeconds(0.2)
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}