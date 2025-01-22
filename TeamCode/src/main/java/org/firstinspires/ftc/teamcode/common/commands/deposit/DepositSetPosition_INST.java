package org.firstinspires.ftc.teamcode.common.commands.deposit;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;

public class DepositSetPosition_INST extends InstantCommand {
   public DepositSetPosition_INST(DepositSubsystem depositSubsystem, DepositSubsystem.BucketState bucketState) {
       super(
               () -> depositSubsystem.setBucketState(bucketState)
       );
   }
}
