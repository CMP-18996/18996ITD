package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-12, 64, Math.toRadians(-90)))
                .splineTo(new Vector2d(-12, 39), Math.toRadians(-90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-39.5, 46, Math.toRadians(245)), Math.toRadians(65))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-38, 53, Math.toRadians(150)), Math.toRadians(150))
                .setReversed(false)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-41, 51, Math.toRadians(235)), Math.toRadians(55))
                .setReversed(true)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-38, 53, Math.toRadians(145)), Math.toRadians(145))
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}