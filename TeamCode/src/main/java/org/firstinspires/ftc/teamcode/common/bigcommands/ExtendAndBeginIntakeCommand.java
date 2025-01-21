package org.firstinspires.ftc.teamcode.common.bigcommands;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commands.ExtendCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class ExtendAndBeginIntakeCommand extends SequentialCommandGroup {
    public ExtendAndBeginIntakeCommand(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem, LiftSubsystem liftSubsystem) {
        addCommands(
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.PICKING_UP),
                new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.ACTIVE),
                new ParallelDeadlineGroup(
                        new WaitCommand(800),
                        new LiftSetPosition(liftSubsystem, LiftSubsystem.LiftState.GROUND),
                        new ExtendCommand(extensionSubsystem, ExtensionSubsystem.ExtensionState.FULLY_EXTENDED)
                )
        );
    }
}
