package org.firstinspires.ftc.teamcode.common.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends InstantCommand {
    public IntakeCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.IntakingState intakingState) {
        super(
                () -> intakeSubsystem.updateIntakingState(intakingState)
        );
    }
}
