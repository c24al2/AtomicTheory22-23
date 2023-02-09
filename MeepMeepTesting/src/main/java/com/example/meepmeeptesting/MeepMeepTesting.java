package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static Pose2d START_POSE = new Pose2d(36, -62.5, Math.toRadians(90));
    public static Pose2d STACK_POSE = new Pose2d(60, -12, Math.toRadians(0));
    public static Pose2d PLACE_CONE_POSE = new Pose2d(30, -5, Math.toRadians(135));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(270), Math.toRadians(90), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(START_POSE)
//                                .addTemporalMarker(() -> intake.setClawPosition(IntakeConstants.CLAW_CLOSED_POSITION))
//                                .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.HIGH_JUNCTION_HEIGHT))
                                .splineTo(new Vector2d(36, -24), Math.toRadians(90))
                                .splineTo(PLACE_CONE_POSE.vec(), PLACE_CONE_POSE.getHeading())
//                                .addTemporalMarker(() -> intake.followMotionProfileAsync(IntakeConstants.HIGH_JUNCTION_HEIGHT - IntakeConstants.ON_JUNCTION_HEIGHT_CHANGE))
                                .waitSeconds(0.2)
//                                .addTemporalMarker(() -> intake.setClawPosition(IntakeConstants.CLAW_OPEN_POSITION))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}