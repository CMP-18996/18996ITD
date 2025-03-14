package org.firstinspires.ftc.teamcode.common.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class IntakePivotSetPosition_INST extends InstantCommand {
    public IntakePivotSetPosition_INST(IntakeSubsystem intakeSubsystem, IntakeSubsystem.IntakePivotState intakePivotState) {
        super(
                () -> intakeSubsystem.setIntakePivotState(intakePivotState)
        );
    }
}
