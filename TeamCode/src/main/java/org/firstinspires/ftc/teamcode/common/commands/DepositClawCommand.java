package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;


public class DepositClawCommand extends InstantCommand{
    public DepositClawCommand(DepositSubsystem depositSubsystem, DepositSubsystem.ClawState clawState) {
        super(
                () -> depositSubsystem.updateTransferClawState(clawState)
        );
    }
}
