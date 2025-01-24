package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.commands.autoCommands.AutoDeposit;
import org.firstinspires.ftc.teamcode.common.commands.autoCommands.AutoExtend;
import org.firstinspires.ftc.teamcode.common.commands.autoCommands.AutoExtendRetractTransfer;
import org.firstinspires.ftc.teamcode.common.commands.autoCommands.AutoRetractTransfer;
import org.firstinspires.ftc.teamcode.common.commands.autoCommands.RRWrapper;
import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.Team;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@Autonomous(name="basket auto")
public class BasketAutoRR extends CommandOpMode {
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
                new InstantCommand(() -> robot.extension.setMaxExtensionSpeed(0.6)),
                new SequentialCommandGroup(
                        //deposit preloaded block in basket
                        new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.REST),
                        new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.REST),
                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(64,64, Math.toRadians(225)), Math.toRadians(45))
                        .       build()),
                        new AutoDeposit(robot.lift, robot.deposit),

                        //get first block
                        new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.MOVING),
                        new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.MOVING),
                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(57.5,52, Math.toRadians(-115)), Math.toRadians(-115))
                                .build()),
                        new AutoExtend(robot.extension, robot.intake, robot.lift),

                        //high basket first block
                        new ParallelCommandGroup(
                                new AutoRetractTransfer(robot.extension, robot.intake, robot.deposit, robot.lift),
                                new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(64,64, Math.toRadians(225)), Math.toRadians(45))
                                        .build())
                        ),
                        new AutoDeposit(robot.lift, robot.deposit),

                        //get second block
                        new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.MOVING),
                        new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.MOVING),
                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(55,52, Math.toRadians(-70)), Math.toRadians(-70))
                                .build()),
                        new AutoExtendRetractTransfer(robot.extension, robot.intake, robot.lift, robot.deposit),

                        //high basket second block
                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(64,64, Math.toRadians(225)), Math.toRadians(45))
                                .build()),
                        new AutoDeposit(robot.lift, robot.deposit),

                        //get third block
                        new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.MOVING),
                        new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.MOVING),
                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(63,52, Math.toRadians(-70)), Math.toRadians(-70))
                                .build()),
                        new AutoExtendRetractTransfer(robot.extension, robot.intake, robot.lift, robot.deposit),

                        //high basket third block
                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(64,64, Math.toRadians(225)), Math.toRadians(45))
                                .build()),
                        new AutoDeposit(robot.lift, robot.deposit),

                        //skedaddle
                        /*new InstantCommand(() -> robot.hang.hangMotor.setPower(1)),
                        new InstantCommand(() -> Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .splineTo(new Vector2d(28,16), Math.toRadians(-90))
                                .build())),
                        new InstantCommand(() -> robot.hang.hangMotor.setPower(0))*/

                        //zero stuff
                        new ExtensionSetPosition(robot.extension, ExtensionSubsystem.ExtensionState.TRANSFER),
                        new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.TRANSFER),
                        new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.TRANSFER)
                )
        );
    }
}
