package org.firstinspires.ftc.teamcode.common.commands.complexCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositTrapdoorPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetMotorState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class RetractAndTransferCommand extends SequentialCommandGroup {
    public RetractAndTransferCommand(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem, DepositSubsystem depositSubsystem, LiftSubsystem liftSubsystem) {
        addCommands(
                // raise the intake to clear submersible wall
                new IntakeSetMotorState_INST(intakeSubsystem, IntakeSubsystem.IntakeMotorState.HOLD),
                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.REST),
                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.REST),
                new WaitCommand(100),

                // bring lift and extension into position
                new ParallelCommandGroup(
                        new LiftSetPosition(liftSubsystem, LiftSubsystem.LiftState.TRANSFER),
                        new ExtensionSetPosition(extensionSubsystem, ExtensionSubsystem.ExtensionState.TRANSFER),
                        new DepositSetPosition_INST(depositSubsystem, DepositSubsystem.BucketState.TRANSFER),
                        new DepositTrapdoorPosition_INST(depositSubsystem, DepositSubsystem.DepositTrapdoorState.TOP_OPEN)
                ),

                new WaitCommand(500),

                new TransferSampleCommand(intakeSubsystem)
        );
    }
}
