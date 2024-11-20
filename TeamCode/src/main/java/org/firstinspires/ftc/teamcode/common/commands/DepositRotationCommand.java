package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;

public class DepositRotationCommand extends InstantCommand {
   public DepositRotationCommand(DepositSubsystem depositSubsystem, DepositSubsystem.TransferRotatorState transferRotatorState) {
       super(
               () -> depositSubsystem.updateTransferRotatorState(transferRotatorState)
       );
   }
}
