package org.firstinspires.ftc.teamcode.common.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class IntakeArmPivotCommand extends InstantCommand {
    public IntakeArmPivotCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.IntakeArmPivotState intakeArmPivotState) {
        super(
                () -> intakeSubsystem.updateIntakeArmPivotState(intakeArmPivotState)
        );
    }
}
