package org.firstinspires.ftc.teamcode.common.commands.autoCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commands.complexCommands.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.WaitForColorCommand;
import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetRollerState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeTrapdoorSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class AutoRetractTransfer extends SequentialCommandGroup {
    public AutoRetractTransfer(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem, DepositSubsystem depositSubsystem, LiftSubsystem liftSubsystem) {
        addCommands(
                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.TRANSFER),
                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.TRANSFER),
                new ExtensionSetPosition_INST(extensionSubsystem, ExtensionSubsystem.ExtensionState.TRANSFER),
                new IntakeTrapdoorSetPosition_INST(intakeSubsystem, IntakeSubsystem.TrapdoorState.OPEN),
                new IntakeSetRollerState_INST(intakeSubsystem, IntakeSubsystem.IntakeRollerState.ACTIVE),
                new WaitCommand(500),
                new DepositSetPosition_INST(depositSubsystem, DepositSubsystem.BucketState.TRANSFER),
                new LiftSetPosition_INST(liftSubsystem, LiftSubsystem.LiftState.TRANSFER),
                new WaitCommand(300),

                // wait until no sample is detected
                new ParallelRaceGroup(
                        new WaitForColorCommand(intakeSubsystem, IntakeSubsystem.Color.NONE),
                        new WaitCommand(2000)
                ),
                new WaitCommand(300),
                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.MOVING),
                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.MOVING),
                new IntakeSetRollerState_INST(intakeSubsystem, IntakeSubsystem.IntakeRollerState.DISABLED),
                new IntakeTrapdoorSetPosition_INST(intakeSubsystem, IntakeSubsystem.TrapdoorState.CLOSED)
        );
    }
}
