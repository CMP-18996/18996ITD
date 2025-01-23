package org.firstinspires.ftc.teamcode.common.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class IntakeWristSetPosition_INST extends InstantCommand {
    public IntakeWristSetPosition_INST(IntakeSubsystem intakeSubsystem, IntakeSubsystem.IntakeWristState intakeWristState) {
        super(
                () -> intakeSubsystem.setIntakeWristState(intakeWristState)
        );
    }
}
