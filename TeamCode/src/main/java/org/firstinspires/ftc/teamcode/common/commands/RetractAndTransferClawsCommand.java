package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class RetractAndTransferClawsCommand extends SequentialCommandGroup {
    public RetractAndTransferClawsCommand(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem, DepositSubsystem depositSubsystem, LiftSubsystem liftSubsystem) {
        addCommands(
                new DepositRotationCommand(depositSubsystem, DepositSubsystem.TransferRotatorState.TRANSFER_READY),
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                new ExtendCommand(extensionSubsystem, ExtensionSubsystem.ExtensionState.TRANSFER),
                new TransferClawCommand(depositSubsystem, DepositSubsystem.ClawState.OPEN),
                new WaitCommand(700),
                new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.REVERSING),
                new WaitCommand(300),
                new TransferClawCommand(depositSubsystem, DepositSubsystem.ClawState.CLOSED),
                new WaitCommand(700),
                new InstantLiftCommand(liftSubsystem, LiftSubsystem.HIGH_BASKET),
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.MOVING),
                new DepositRotationCommand(depositSubsystem, DepositSubsystem.TransferRotatorState.READY_TO_DEPOSIT),
                new ExtendCommand(extensionSubsystem, ExtensionSubsystem.ExtensionState.CONTRACTED),
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.TRANSFERRING)
                //new ExtendCommand(extensionSubsystem, ExtensionSubsystem.ExtensionState.CUSTOM)
        );
    }
}
