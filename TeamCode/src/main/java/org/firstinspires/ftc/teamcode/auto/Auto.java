package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.commands.DepositRotationCommand;
import org.firstinspires.ftc.teamcode.common.commands.ExtendAndBeginIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.ExtensionMotorPowerCommand;
import org.firstinspires.ftc.teamcode.common.commands.ExtensionPositionCommand;
import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.RetractAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@Autonomous(name="auto for now")
public class Auto extends CommandOpMode {
    Pose2d beginPose;
    SparkFunOTOSDrive drive;
    Robot robot;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        beginPose = new Pose2d(66, 66, Math.toRadians(-135));
        drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        robot = new Robot(hardwareMap, Subsystems.EXTENSION, Subsystems.INTAKE, Subsystems.LIFT, Subsystems.DEPOSIT);

        super.schedule(new InstantCommand(),
                new SequentialCommandGroup(
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .splineTo(new Vector2d(60,52), Math.toRadians(-90))
                                .build())),
                        new ExtendAndBeginIntakeCommand(robot.extension, robot.intake, robot.lift),
                        new WaitCommand(2000),
                        new RetractAndTransferCommand(robot.extension, robot.intake, robot.deposit),
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .splineTo(new Vector2d(66,66), Math.toRadians(45))
                                .build())),
                        new LiftSetPosition(robot.lift, LiftSubsystem.HIGH_BASKET),
                        new WaitCommand(2000),
                        new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.DEPOSITING),
                        new WaitCommand(5000),
                        new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY)
        ));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
