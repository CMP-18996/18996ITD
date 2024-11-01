package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;

public class DepositRotationCommand extends CommandBase {
    DepositSubsystem deposit;
    DepositSubsystem.TransferRotatorState transferState;
    public DepositRotationCommand(DepositSubsystem deposit, DepositSubsystem.TransferRotatorState transferState) {
        this.deposit = deposit;
        this.transferState = transferState;
    }

    public void initialize() {
        deposit.updateTransferRotatorState(transferState);
    }
}
