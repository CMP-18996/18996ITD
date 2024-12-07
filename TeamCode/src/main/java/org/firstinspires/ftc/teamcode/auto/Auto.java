package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.commands.DepositRotationCommand;
import org.firstinspires.ftc.teamcode.common.commands.ExtendAndBeginIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.ExtensionMotorPowerCommand;
import org.firstinspires.ftc.teamcode.common.commands.ExtensionPositionCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.RetractAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.Team;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@Autonomous(name="basket auto")
public class Auto extends CommandOpMode {
    Pose2d beginPose;
    SparkFunOTOSDrive drive;
    Robot robot;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        beginPose = new Pose2d(43, 68, Math.toRadians(-180));
        drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        robot = new Robot(hardwareMap, Team.BLUE, Subsystems.EXTENSION, Subsystems.INTAKE, Subsystems.LIFT, Subsystems.DEPOSIT);

        super.schedule(
                new SequentialCommandGroup(
                        //get first block
                        new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .splineTo(new Vector2d(64,64), Math.toRadians(45))
                        .       build())),

                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .splineTo(new Vector2d(51,53), Math.toRadians(-90))
                                .build())),
                        new ExtendAndBeginIntakeCommand(robot.extension, robot.intake, robot.lift),
                        new WaitCommand(500),
                        new RetractAndTransferCommand(robot.extension, robot.intake, robot.deposit),

                        //high basket first block
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .splineTo(new Vector2d(64,64), Math.toRadians(45))
                                .build())),
                        new ParallelDeadlineGroup(
                                new WaitCommand(800),
                                new LiftSetPosition(robot.lift, LiftSubsystem.HIGH_BASKET),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.DEPOSITING)
                        ),
                        new ParallelDeadlineGroup(
                                new WaitCommand(700),
                                new LiftSetPosition(robot.lift, LiftSubsystem.GROUND),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY)
                        ),

                        //get second block
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .splineTo(new Vector2d(61,53), Math.toRadians(-90))
                                .build())),
                        new ExtendAndBeginIntakeCommand(robot.extension, robot.intake, robot.lift),
                        new WaitCommand(500),
                        new RetractAndTransferCommand(robot.extension, robot.intake, robot.deposit),

                        //high basket second block
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .splineTo(new Vector2d(64,64), Math.toRadians(45))
                                .build())),
                        new ParallelDeadlineGroup(
                                new WaitCommand(800),
                                new LiftSetPosition(robot.lift, LiftSubsystem.HIGH_BASKET),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.DEPOSITING)
                        ),
                        new ParallelDeadlineGroup(
                                new WaitCommand(700),
                                new LiftSetPosition(robot.lift, LiftSubsystem.GROUND),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY)
                        ),

                        //get third block
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .splineTo(new Vector2d(61,50), Math.toRadians(-65))
                                .build())),
                        //unique to this repeat - may implement to others dep. on testing
                        new ParallelDeadlineGroup(
                                new WaitCommand(800),
                                new ExtendAndBeginIntakeCommand(robot.extension, robot.intake, robot.lift)
                        ),
                        new WaitCommand(500),
                        new RetractAndTransferCommand(robot.extension, robot.intake, robot.deposit),

                        //high basket third block
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .splineTo(new Vector2d(64,64), Math.toRadians(45))
                                .build())),
                        new ParallelDeadlineGroup(
                                new WaitCommand(800),
                                new LiftSetPosition(robot.lift, LiftSubsystem.HIGH_BASKET),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.DEPOSITING)
                        ),
                        new ParallelDeadlineGroup(
                                new WaitCommand(700),
                                new LiftSetPosition(robot.lift, LiftSubsystem.GROUND),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY)
                        ),

                        //park
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .splineTo(new Vector2d(28,16), Math.toRadians(180))
                                .build()))
                )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
