package org.firstinspires.ftc.teamcode.common.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class IntakeClawSetState_INST extends InstantCommand {
    public IntakeClawSetState_INST(IntakeSubsystem intakeSubsystem, IntakeSubsystem.IntakeClawState intakeClawState) {
        super(
                () -> intakeSubsystem.setIntakeClawState(intakeClawState)
        );
    }
}
