package org.firstinspires.ftc.teamcode.common.commands.complexCommands;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositTrapdoorPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetMotorState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeTrapdoorSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class HumanPlayerDepositCommand extends SequentialCommandGroup {
    public HumanPlayerDepositCommand(DepositSubsystem depositSubsystem, LiftSubsystem liftSubsystem) {
        addCommands(
                new LiftSetPosition(liftSubsystem, LiftSubsystem.LiftState.HUMAN_PLAYER_DEPOSIT),
                new DepositSetPosition_INST(depositSubsystem, DepositSubsystem.BucketState.HUMAN_PLAYER_DEPOSIT),
                new WaitCommand(500),
                new DepositTrapdoorPosition_INST(depositSubsystem, DepositSubsystem.DepositTrapdoorState.BOTTOM_OPEN),
                new WaitCommand(1000),
                new DepositSetPosition_INST(depositSubsystem, DepositSubsystem.BucketState.TRANSFER),
                new DepositTrapdoorPosition_INST(depositSubsystem, DepositSubsystem.DepositTrapdoorState.TOP_OPEN)
        );
    }
}
