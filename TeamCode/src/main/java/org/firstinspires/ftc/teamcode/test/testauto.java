package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.commands.autoCommands.RRWrapper;
import org.firstinspires.ftc.teamcode.common.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.Team;

@Autonomous(name="auto testing")
public class testauto extends CommandOpMode {
    Pose2d beginPose;
    SparkFunOTOSDrive drive;
    Robot robot;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        beginPose = new Pose2d(-12, 64, Math.toRadians(-90));
        drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        robot = new Robot(hardwareMap, Team.BLUE);

        super.schedule(
                new SequentialCommandGroup(
                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .splineTo(new Vector2d(-12, 38), Math.toRadians(-90))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-40, 46, Math.toRadians(240)), Math.toRadians(70))
                                .turn(Math.toRadians(-110))
                                .build())
                )
        );
    }
    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
