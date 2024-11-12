package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class ExtendAndBeginIntakeCommand extends SequentialCommandGroup {
    public ExtendAndBeginIntakeCommand(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem, LiftSubsystem liftSubsystem) {
        addCommands(
                new LiftSetPosition(liftSubsystem, 0),
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.MOVING),
                new ExtendCommand(extensionSubsystem, ExtensionSubsystem.ExtensionState.FULLY_EXTENDED),
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.PICKING_UP),
                new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.ACTIVE)
        );
    }
}
