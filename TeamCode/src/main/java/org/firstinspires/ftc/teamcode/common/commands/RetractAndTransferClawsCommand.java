package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class RetractAndTransferClawsCommand extends SequentialCommandGroup {
    public RetractAndTransferClawsCommand(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem, DepositSubsystem depositSubsystem) {
        addCommands(
                //new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.DISABLED),
                new ExtendCommand(extensionSubsystem, ExtensionSubsystem.ExtensionState.CONTRACTED),
                new DepositRotationCommand(depositSubsystem, DepositSubsystem.TransferRotatorState.TRANSFER_READY),
                new TransferClawCommand(depositSubsystem, DepositSubsystem.ClawState.OPEN),
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                new WaitCommand(700),
                new ParallelCommandGroup(
                    new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.REVERSING),
                    new TransferClawCommand(depositSubsystem, DepositSubsystem.ClawState.CLOSED)
                ),
                new WaitCommand(700),
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.MOVING)
        );
    }
}
