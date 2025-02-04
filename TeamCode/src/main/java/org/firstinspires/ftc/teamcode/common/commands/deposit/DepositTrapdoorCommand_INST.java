package org.firstinspires.ftc.teamcode.common.commands.deposit;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;

public class DepositTrapdoorCommand_INST extends InstantCommand {
    DepositSubsystem.DepositTrapdoorState state;
    DepositSubsystem deposit;
    public DepositTrapdoorCommand_INST(DepositSubsystem deposit, DepositSubsystem.DepositTrapdoorState state) {
        this.state = state;
        this.deposit = deposit;
    }

    @Override
    public void initialize() {
        deposit.setDepositTrapdoor(state);
    }
}
