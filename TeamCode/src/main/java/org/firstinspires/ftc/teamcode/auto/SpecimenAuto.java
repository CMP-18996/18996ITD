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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.common.autocmd.AutoExtend;
import org.firstinspires.ftc.teamcode.common.autocmd.RRWrapper;
import org.firstinspires.ftc.teamcode.common.commands.DepositRotationCommand;
import org.firstinspires.ftc.teamcode.common.commands.ExtendAndBeginIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.ExtendCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.SpecimenArmCommand;
import org.firstinspires.ftc.teamcode.common.commands.SpecimenGripperCommand;
import org.firstinspires.ftc.teamcode.common.commands.ZeroMotorCommand;
import org.firstinspires.ftc.teamcode.common.drive.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.Team;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
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
                        new LiftSetPosition(robot.lift, LiftSubsystem.GROUND),
                        new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.CLOSED),
                        new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                        new WaitCommand(50),
                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .splineTo(new Vector2d(-12, 38), Math.toRadians(-90))
                                .setReversed(true)
                                .afterDisp(2, () -> super.schedule(new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.OPEN), new SpecimenArmCommand(robot.specimen, SpecimenSubsystem.SpecimenPosition.WALL)))
                                .splineToLinearHeading(new Pose2d(-40, 46, Math.toRadians(240)), Math.toRadians(60))
                                .build()),

                        //move blocks over
                        new AutoExtend(robot.extension, robot.intake, robot.lift),
                        new WaitCommand(500),
                        new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED),
                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .afterTime(0.5, () -> super.schedule(new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING)))
                                .turn(Math.toRadians(-110))
                                .build()),

                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .afterDisp(0.1, () -> super.schedule(new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.CONTRACTED)))
                                .splineToLinearHeading(new Pose2d(-48, 48, Math.toRadians(240)), Math.toRadians(240))
                                .build()),
                        new AutoExtend(robot.extension, robot.intake, robot.lift),
                        new WaitCommand(500),
                        new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED),
                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .afterTime(0.5, () -> super.schedule(new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING)))
                                .turn(Math.toRadians(-100))
                                .build()),

                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .afterDisp(0.1, () -> super.schedule(new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.CONTRACTED)))
                                .splineToLinearHeading(new Pose2d(-58, 48, Math.toRadians(240)), Math.toRadians(240))
                                .build()),
                        new AutoExtend(robot.extension, robot.intake, robot.lift),
                        new WaitCommand(500),
                        new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED),
                        new RRWrapper(drive, drive.actionBuilder(drive.pose)
                                .afterTime(0.5, () -> super.schedule(new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING)))
                                .turn(Math.toRadians(179))
                                .build())
                )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
