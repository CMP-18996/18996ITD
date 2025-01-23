package org.firstinspires.ftc.teamcode.common.commands.autoCommands;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class AutoDeposit extends SequentialCommandGroup {
    public AutoDeposit(LiftSubsystem liftSubsystem, DepositSubsystem depositSubsystem) {
        addCommands(
                new ParallelDeadlineGroup(
                        new WaitCommand(1000),
                        new LiftSetPosition(liftSubsystem, LiftSubsystem.LiftState.HIGH_BUCKET),
                        new SequentialCommandGroup(
                                new WaitCommand(400),
                                new DepositSetPosition_INST(depositSubsystem, DepositSubsystem.BucketState.DEPOSIT)
                        )
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(800),
                        new LiftSetPosition(liftSubsystem, LiftSubsystem.LiftState.TRANSFER),
                        new DepositSetPosition_INST(depositSubsystem, DepositSubsystem.BucketState.TRANSFER)
                )
        );
    }
}
