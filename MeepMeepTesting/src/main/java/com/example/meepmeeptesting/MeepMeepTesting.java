package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(43, 68, Math.toRadians(-180)))
                        .setReversed(true)
                        .splineTo(new Vector2d(64,64), Math.toRadians(45))
                        .setReversed(false)
                        .splineTo(new Vector2d(58.75,49), Math.toRadians(-110))
                        .setReversed(true)
                        .splineTo(new Vector2d(64,64), Math.toRadians(45))
                        .setReversed(false)
                        .splineTo(new Vector2d(57.5,47.5), Math.toRadians(-70))
                        .setReversed(true)
                        .splineTo(new Vector2d(64,64), Math.toRadians(45))
                        .setReversed(false)
                        .splineTo(new Vector2d(66.5,50), Math.toRadians(-70))
                        .setReversed(true)
                        .splineTo(new Vector2d(64,64), Math.toRadians(45))
                        .setReversed(false)
                        .splineTo(new Vector2d(43,68), Math.toRadians(-180))
                        .splineTo(new Vector2d(-60,68), Math.toRadians(-180))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}