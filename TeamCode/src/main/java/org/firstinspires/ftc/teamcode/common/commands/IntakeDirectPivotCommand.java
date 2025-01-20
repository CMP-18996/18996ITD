package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class IntakeDirectPivotCommand extends InstantCommand {
    public IntakeDirectPivotCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.IntakeDirectPivotState intakeDirectPivotState) {
        super(
                () -> intakeSubsystem.updateIntakeDirectPivotState(intakeDirectPivotState)
        );
    }
}
