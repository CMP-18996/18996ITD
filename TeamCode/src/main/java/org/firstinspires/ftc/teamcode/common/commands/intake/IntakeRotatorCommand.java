package org.firstinspires.ftc.teamcode.common.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class IntakeRotatorCommand extends InstantCommand {
    public IntakeRotatorCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.IntakeRotatorState intakeRotatorState) {
        super(
                () -> intakeSubsystem.updateIntakeRotatorState(intakeRotatorState)
        );
    }
}
