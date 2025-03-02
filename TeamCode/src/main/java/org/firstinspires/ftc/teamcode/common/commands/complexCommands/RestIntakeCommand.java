package org.firstinspires.ftc.teamcode.common.commands.complexCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeClawSetState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakePivotSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeRollerSetState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class RestIntakeCommand extends SequentialCommandGroup {
    public RestIntakeCommand(IntakeSubsystem intakeSubsystem) {
        addCommands(
                new IntakeRollerSetState_INST(intakeSubsystem, IntakeSubsystem.IntakeRollerState.DISABLED),
                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.REST),
                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.REST),
                new IntakePivotSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakePivotState.PIVOT_0),
                new IntakeClawSetState_INST(intakeSubsystem, IntakeSubsystem.IntakeClawState.CLOSED)
        );
    }
}
