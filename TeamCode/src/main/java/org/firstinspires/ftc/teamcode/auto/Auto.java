package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.drive.SparkFunOTOSDrive;

@Autonomous(name="auto for now")
public class Auto extends OpMode {
    Pose2d beginPose;
    SparkFunOTOSDrive drive;
    @Override
    public void init() {
        beginPose = new Pose2d(36, 57, Math.toRadians(0));
        drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
    }
    @Override
    public void start() {
        Actions.runBlocking(drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(54,54), Math.toRadians(45))
                .setReversed(true)
                .splineTo(new Vector2d(46,46), Math.toRadians(-90))
                .setReversed(false)
                .splineTo(new Vector2d(54,54), Math.toRadians(45))
                .setReversed(true)
                .splineTo(new Vector2d(0, 36), Math.toRadians(-90))
                .build());
    }
    @Override
    public void loop() {

    }
}
