package org.firstinspires.ftc.teamcode.common.commands.deposit;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;

public class DepositTrapdoorPosition_INST extends InstantCommand {
    public DepositTrapdoorPosition_INST(DepositSubsystem depositSubsystem, DepositSubsystem.DepositTrapdoorState depositTrapdoorState) {
        super(
                () -> depositSubsystem.setDepositTrapdoorState(depositTrapdoorState)
        );
    }
}
