package org.firstinspires.ftc.teamcode.common.commands.complexCommands;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetMotorState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class RejectSampleCommand extends SequentialCommandGroup {
    public RejectSampleCommand(IntakeSubsystem intakeSubsystem) {
        addCommands(
                new IntakeSetMotorState_INST(intakeSubsystem, IntakeSubsystem.IntakeMotorState.DISABLED),
                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.REJECT),
                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.REJECT),
                new WaitCommand(500),
                new IntakeSetMotorState_INST(intakeSubsystem, IntakeSubsystem.IntakeMotorState.REVERSING),

                /*
                new ParallelRaceGroup(
                    new WaitForColorCommand(intakeSubsystem, IntakeSubsystem.Color.NONE),
                    new WaitCommand(1000)
                ),

                 */
                new WaitCommand(500),

                new IntakeSetMotorState_INST(intakeSubsystem, IntakeSubsystem.IntakeMotorState.DISABLED)
        );
    }
}
