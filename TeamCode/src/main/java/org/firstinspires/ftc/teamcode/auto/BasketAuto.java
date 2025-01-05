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

import org.firstinspires.ftc.teamcode.common.autocmd.AutoDeposit;
import org.firstinspires.ftc.teamcode.common.autocmd.AutoExtendRetractTransfer;
import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.commands.ZeroMotorCommand;
import org.firstinspires.ftc.teamcode.common.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.Team;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@Autonomous(name="basket auto")
public class BasketAuto extends CommandOpMode {
    Pose2d beginPose;
    SparkFunOTOSDrive drive;
    Robot robot;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        beginPose = new Pose2d(43, 68, Math.toRadians(-180));
        drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        robot = new Robot(hardwareMap, Team.BLUE, false, Subsystems.EXTENSION, Subsystems.INTAKE, Subsystems.LIFT, Subsystems.DEPOSIT);

        super.schedule(
                new InstantCommand(() -> robot.extension.setMaxPower(0.6)),
                new SequentialCommandGroup(
                        //deposit preloaded block in basket bc specimen arm is broken
                        new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .splineTo(new Vector2d(64,64), Math.toRadians(45))
                        .       build())),
                        new AutoDeposit(robot.lift, robot.deposit),

                        //get first block
                        new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.MOVING),
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .splineTo(new Vector2d(58.75,49), Math.toRadians(-110))
                                .build())),
                        new AutoExtendRetractTransfer(robot.extension, robot.intake, robot.lift, robot.deposit),

                        //high basket first block
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .splineTo(new Vector2d(64,64), Math.toRadians(45))
                                .build())),
                        new AutoDeposit(robot.lift, robot.deposit),

                        //get second block
                        new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.MOVING),
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .splineTo(new Vector2d(57.5,48), Math.toRadians(-70))
                                .build())),
                        new AutoExtendRetractTransfer(robot.extension, robot.intake, robot.lift, robot.deposit),

                        //high basket second block
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .splineTo(new Vector2d(64,64), Math.toRadians(45))
                                .build())),
                        new AutoDeposit(robot.lift, robot.deposit),

                        //get third block
                        new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.MOVING),
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .splineTo(new Vector2d(65.5,49), Math.toRadians(-65))
                                .build())),
                        new AutoExtendRetractTransfer(robot.extension, robot.intake, robot.lift, robot.deposit),

                        //high basket third block
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .splineTo(new Vector2d(64,64), Math.toRadians(45))
                                .build())),
                        new AutoDeposit(robot.lift, robot.deposit)

                        //skedaddle
                        /*new InstantCommand(() -> robot.hang.hangMotor.setPower(1)),
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .splineTo(new Vector2d(28,16), Math.toRadians(-90))
                                .build())),
                        new InstantCommand(() -> robot.hang.hangMotor.setPower(0))*/

                        //zero stuff
                )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
