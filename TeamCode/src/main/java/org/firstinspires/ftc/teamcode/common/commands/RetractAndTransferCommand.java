package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
public class RetractAndTransferCommand extends SequentialCommandGroup {
    public RetractAndTransferCommand(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem, DepositSubsystem depositSubsystem) {
        addCommands(
                //new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.DISABLED),
                new IntakeDirectPivotCommand(intakeSubsystem, IntakeSubsystem.IntakeDirectPivotState.MOVING),
                new IntakeArmPivotCommand(intakeSubsystem, IntakeSubsystem.IntakeArmPivotState.MOVING),
                new DepositRotationCommand(depositSubsystem, DepositSubsystem.TransferRotatorState.TRANSFER_READY),
                new ExtendCommand(extensionSubsystem, ExtensionSubsystem.ExtensionState.CONTRACTED),
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                new IntakeArmPivotCommand(intakeSubsystem, IntakeSubsystem.IntakeArmPivotState.TRANSFERRING),
                new WaitCommand(700),
                new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.REVERSING),
                new WaitCommand(700),
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.MOVING)
        );
    }
}
