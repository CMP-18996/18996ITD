package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.autocmd.AutoExtend;
import org.firstinspires.ftc.teamcode.common.commands.DepositRotationCommand;
import org.firstinspires.ftc.teamcode.common.commands.ExtendAndBeginIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.SpecimenArmCommand;
import org.firstinspires.ftc.teamcode.common.commands.SpecimenGripperCommand;
import org.firstinspires.ftc.teamcode.common.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.Team;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@Autonomous(name="specimen auto")
public class SpecimenAuto extends CommandOpMode {
    Pose2d beginPose;
    SparkFunOTOSDrive drive;
    Robot robot;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        beginPose = new Pose2d(-12, 66, Math.toRadians(-90));
        drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        robot = new Robot(hardwareMap, Team.BLUE, Subsystems.EXTENSION, Subsystems.INTAKE, Subsystems.SPECIMEN, Subsystems.LIFT);

        super.schedule(
                new SequentialCommandGroup(
                        //drop preloaded specimen
                        new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.CLOSED),
                        new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                        new SpecimenArmCommand(robot.specimen, SpecimenSubsystem.SpecimenPosition.CHAMBER),
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineTo(new Vector2d(-12, 36), Math.toRadians(-90))
                                .build())),
                        new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.OPEN),

                        //move blocks over
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-36, 48, Math.toRadians(240)), Math.toRadians(70))
                                .build())),
                        new AutoExtend(robot.extension, robot.intake, robot.lift),
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .turn(Math.toRadians(-90))
                                .build())),
                        new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING),
                        new WaitCommand(500),
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-48, 48, Math.toRadians(240)), Math.toRadians(240))
                                .build())),
                        new AutoExtend(robot.extension, robot.intake, robot.lift),
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .turn(Math.toRadians(-100))
                                .build())),
                        new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING),
                        new WaitCommand(500),
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineToLinearHeading(new Pose2d(-58, 48, Math.toRadians(240)), Math.toRadians(240))
                                .build())),
                        new AutoExtend(robot.extension, robot.intake, robot.lift),
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .turn(Math.toRadians(179))
                                .build())),
                        new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING),
                        new WaitCommand(500),

                        //yoink specimen
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .turn(Math.toRadians(179))
                                .build()))
                )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
