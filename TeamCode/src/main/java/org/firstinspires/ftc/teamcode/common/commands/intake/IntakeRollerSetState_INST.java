package org.firstinspires.ftc.teamcode.common.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class IntakeRollerSetState_INST extends InstantCommand {
    public IntakeRollerSetState_INST(IntakeSubsystem intakeSubsystem, IntakeSubsystem.IntakeRollerState intakeRollerState) {
        super(
                () -> intakeSubsystem.setIntakeRollerState(intakeRollerState)
        );
    }
}
