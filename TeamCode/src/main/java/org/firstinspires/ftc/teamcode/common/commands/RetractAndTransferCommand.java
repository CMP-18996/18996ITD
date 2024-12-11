package org.firstinspires.ftc.teamcode.common.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
public class RetractAndTransferCommand extends SequentialCommandGroup {
    public RetractAndTransferCommand(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem, DepositSubsystem depositSubsystem) {
        addCommands(
                //new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.DISABLED),
                new ExtendCommand(extensionSubsystem, ExtensionSubsystem.ExtensionState.CONTRACTED),
                new DepositRotationCommand(depositSubsystem, DepositSubsystem.TransferRotatorState.TRANSFER_READY),
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                new WaitCommand(700),
                new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.REVERSING),
                new WaitCommand(700),
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.MOVING)
        );
    }
}
