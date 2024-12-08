package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class ExtendAndBeginIntakeCommand extends SequentialCommandGroup {
    public ExtendAndBeginIntakeCommand(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem, LiftSubsystem liftSubsystem) {
        addCommands(
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.PICKING_UP),
                new LiftSetPosition(liftSubsystem, LiftSubsystem.GROUND),
                //new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.MOVING),
                new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.ACTIVE),
                new ExtendCommand(extensionSubsystem, ExtensionSubsystem.ExtensionState.FULLY_EXTENDED)
        );
    }
}
