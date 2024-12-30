package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.autocmd.AutoDeposit;
import org.firstinspires.ftc.teamcode.common.autocmd.RRWrapper;
import org.firstinspires.ftc.teamcode.common.commands.ExtendCommand;
import org.firstinspires.ftc.teamcode.common.commands.SpecimenArmCommand;
import org.firstinspires.ftc.teamcode.common.commands.SpecimenGripperCommand;
import org.firstinspires.ftc.teamcode.common.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.Team;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

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
        robot = new Robot(hardwareMap, Team.BLUE, false);

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
