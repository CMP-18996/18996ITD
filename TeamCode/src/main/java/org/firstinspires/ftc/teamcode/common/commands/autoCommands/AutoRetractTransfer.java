package org.firstinspires.ftc.teamcode.common.commands.autoCommands;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commands.complexCommands.RetractAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.WaitForColorCommand;
import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetMotorState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeTrapdoorSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class AutoRetractTransfer extends SequentialCommandGroup {
    public AutoRetractTransfer(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem, DepositSubsystem depositSubsystem, LiftSubsystem liftSubsystem) {
        addCommands(
                new RetractAndTransferCommand(extensionSubsystem, intakeSubsystem, depositSubsystem, liftSubsystem),

                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.MOVING),
                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.MOVING),
                new IntakeTrapdoorSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeTrapdoorState.CLOSED),
                new IntakeSetMotorState_INST(intakeSubsystem, IntakeSubsystem.IntakeMotorState.DISABLED)
        );
    }
}
