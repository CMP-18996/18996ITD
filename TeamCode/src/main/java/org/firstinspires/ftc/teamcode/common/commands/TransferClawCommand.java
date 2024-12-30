package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;


public class TransferClawCommand extends InstantCommand{
    public TransferClawCommand(DepositSubsystem depositSubsystem, DepositSubsystem.ClawState clawState) {
        super(
                () -> depositSubsystem.updateTransferClawState(clawState)
        );
    }
}
