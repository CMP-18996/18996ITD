package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-12, 66, Math.toRadians(-90)))
                        .splineTo(new Vector2d(-12, 36), Math.toRadians(-90))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-36, 48, Math.toRadians(240)), Math.toRadians(70))
                        .turn(Math.toRadians(-90))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-48, 48, Math.toRadians(240)), Math.toRadians(240))
                        .turn(Math.toRadians(-100))
                        .splineToLinearHeading(new Pose2d(-58, 48, Math.toRadians(240)), Math.toRadians(240))
                        .turn(Math.toRadians(179))
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(-36, 56, Math.toRadians(-90)), Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(-36, 66, Math.toRadians(-90)), Math.toRadians(-90))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}