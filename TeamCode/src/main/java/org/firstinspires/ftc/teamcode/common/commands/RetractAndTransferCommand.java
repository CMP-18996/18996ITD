package org.firstinspires.ftc.teamcode.common.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

@Config
public class RetractAndTransferCommand extends SequentialCommandGroup {
    public static final int REVERSE_INTAKE_TIME = 600;
    public static final int ROTATE_TIME = 400;
    public RetractAndTransferCommand(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem, DepositSubsystem depositSubsystem) {
        addCommands(
                //new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.DISABLED),
                new ExtendCommand(extensionSubsystem, ExtensionSubsystem.ExtensionState.CONTRACTED),
                new DepositRotationCommand(depositSubsystem, DepositSubsystem.TransferRotatorState.TRANSFER_READY),
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                new WaitCommand(ROTATE_TIME),
                new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.REVERSING),
                new WaitCommand(REVERSE_INTAKE_TIME),
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.MOVING)
        );
    }
}
