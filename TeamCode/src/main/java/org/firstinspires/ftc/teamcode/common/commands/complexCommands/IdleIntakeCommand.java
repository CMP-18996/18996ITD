package org.firstinspires.ftc.teamcode.common.commands.complexCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeClawSetState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeRollerSetState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class IdleIntakeCommand extends SequentialCommandGroup {
    public IdleIntakeCommand(IntakeSubsystem intakeSubsystem) {
        addCommands(
                new IntakeRollerSetState_INST(intakeSubsystem, IntakeSubsystem.IntakeRollerState.DISABLED),
                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.MOVING),
                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.MOVING),
                new IntakeClawSetState_INST(intakeSubsystem, IntakeSubsystem.IntakeClawState.CLOSED)
        );
    }
}
