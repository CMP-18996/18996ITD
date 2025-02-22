package org.firstinspires.ftc.teamcode.common.commands.complexCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeClawSetState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakePivotSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeRollerSetState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class HoldSampleCommand extends SequentialCommandGroup {
    public HoldSampleCommand(IntakeSubsystem intakeSubsystem) {
        addCommands(
                new IntakePivotSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakePivotState.PIVOT_0),
                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.EJECT),
                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.EJECT),

                new WaitCommand(500),

                new IntakeRollerSetState_INST(intakeSubsystem, IntakeSubsystem.IntakeRollerState.HOLD)
        );
    }
}
