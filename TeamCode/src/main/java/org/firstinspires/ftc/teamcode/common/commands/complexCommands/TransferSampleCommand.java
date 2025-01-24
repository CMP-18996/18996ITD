package org.firstinspires.ftc.teamcode.common.commands.complexCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetRollerState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeTrapdoorSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

/*
THIS ONLY DOE STHE INTAKE PART USE RETRACT AND TRANSFER COMMAND
 */
public class TransferSampleCommand extends SequentialCommandGroup {
    public TransferSampleCommand(IntakeSubsystem intakeSubsystem) {
        addCommands(
                // raise the intake and reverse rollers
                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.TRANSFER),
                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.TRANSFER),
                new WaitCommand(200),
                new IntakeTrapdoorSetPosition_INST(intakeSubsystem, IntakeSubsystem.TrapdoorState.OPEN),
                new IntakeSetRollerState_INST(intakeSubsystem, IntakeSubsystem.IntakeRollerState.ACTIVE),

                // wait until no sample is detected
                new WaitForColorCommand(intakeSubsystem, IntakeSubsystem.Color.NONE),
                new WaitCommand(300),
                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.REST),
                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.REST),
                new IntakeTrapdoorSetPosition_INST(intakeSubsystem, IntakeSubsystem.TrapdoorState.CLOSED)
        );
    }
}
