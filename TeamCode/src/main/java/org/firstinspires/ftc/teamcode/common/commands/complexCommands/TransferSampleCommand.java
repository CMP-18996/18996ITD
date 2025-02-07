package org.firstinspires.ftc.teamcode.common.commands.complexCommands;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetMotorState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeTrapdoorSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

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
                new IntakeTrapdoorSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeTrapdoorState.OPEN),
                new IntakeSetMotorState_INST(intakeSubsystem, IntakeSubsystem.IntakeMotorState.HOLD),

                // wait until no sample is detected
                new ParallelRaceGroup(
                        new WaitForColorCommand(intakeSubsystem, IntakeSubsystem.Color.NONE),
                        new WaitCommand(2000)
                ),

                new WaitCommand(300),
                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.REST),
                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.REST),
                new IntakeTrapdoorSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeTrapdoorState.CLOSED),
                new IntakeSetMotorState_INST(intakeSubsystem, IntakeSubsystem.IntakeMotorState.DISABLED)
        );
    }
}
