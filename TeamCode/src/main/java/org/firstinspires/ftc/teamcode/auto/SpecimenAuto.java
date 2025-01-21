package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.autocmd.AutoExtend;
import org.firstinspires.ftc.teamcode.common.autocmd.RRWrapper;
import org.firstinspires.ftc.teamcode.common.commands.ExtendCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.SpecimenArmCommand;
import org.firstinspires.ftc.teamcode.common.commands.SpecimenGripperCommand;
import org.firstinspires.ftc.teamcode.common.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.Team;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
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
        beginPose = new Pose2d(-12, 64, Math.toRadians(-90));
        drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        robot = new Robot(hardwareMap, Team.BLUE, false, Subsystems.EXTENSION, Subsystems.INTAKE, Subsystems.SPECIMEN, Subsystems.LIFT);

        super.schedule(
                new SequentialCommandGroup(
                        //drop preloaded specimen
                        new InstantCommand(() -> robot.extension.setMaxPower(0.6)),
                        new SpecimenArmCommand(robot.specimen, SpecimenSubsystem.SpecimenPosition.CHAMBER),
                        new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.CLOSED),
                        new LiftSetPosition(robot.lift, LiftSubsystem.LiftState.GROUND),
                        new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.MOVING),
                        new WaitCommand(500),
                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .splineTo(new Vector2d(-12, 39), Math.toRadians(-90))
                                .setReversed(true)
                                .afterDisp(1, () -> super.schedule(new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.OPEN), new SpecimenArmCommand(robot.specimen, SpecimenSubsystem.SpecimenPosition.WALL)))
                                .afterDisp(35, () -> super.schedule(new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.PICKING_UP)))
                                .splineToLinearHeading(new Pose2d(-39.5, 46, Math.toRadians(245)), Math.toRadians(65))
                                .build()),


                        //move blocks over
                        new AutoExtend(robot.extension, robot.intake, robot.lift),
                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .afterTime(1.5, () -> super.schedule(new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING)))
                                .splineToLinearHeading(new Pose2d(-38, 53, Math.toRadians(150)), Math.toRadians(150))
                                .build()),

                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .afterDisp(0.1, () -> super.schedule(new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.CONTRACTED)))
                                .splineToLinearHeading(new Pose2d(-41, 51, Math.toRadians(235)), Math.toRadians(55))
                                .build()),
                        new AutoExtend(robot.extension, robot.intake, robot.lift),
                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .afterTime(2, () -> super.schedule(new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING)))
                                .splineToLinearHeading(new Pose2d(-38, 53, Math.toRadians(145)), Math.toRadians(145))
                                .build()),

                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .afterDisp(0.1, () -> super.schedule(new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.CONTRACTED)))
                                .splineToLinearHeading(new Pose2d(-45, 51, Math.toRadians(225)), Math.toRadians(45))
                                .build()),
                        new AutoExtend(robot.extension, robot.intake, robot.lift),
                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .afterTime(2.5, () -> super.schedule(new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING)))
                                .splineToLinearHeading(new Pose2d(-38, 53, Math.toRadians(145)), Math.toRadians(145))
                                .build()),

                        //hang specimens
                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .afterDisp(1, () -> super.schedule(new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED), new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.CONTRACTED), new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.TRANSFERRING)))
                                .splineToLinearHeading(new Pose2d(-48, 64, Math.toRadians(-90)), Math.toRadians(90))
                                .build()),
                        new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.CLOSED),
                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .afterDisp(1, () -> super.schedule(new SpecimenArmCommand(robot.specimen, SpecimenSubsystem.SpecimenPosition.CHAMBER)))
                                .splineToConstantHeading(new Vector2d(-24, 56), Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(-12, 39), Math.toRadians(-90))
                                .build()),

                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .afterDisp(1, () -> super.schedule(new SpecimenArmCommand(robot.specimen, SpecimenSubsystem.SpecimenPosition.WALL), new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.OPEN)))
                                .splineToLinearHeading(new Pose2d(-48, 64, Math.toRadians(-90)), Math.toRadians(90))
                                .build())
                )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
