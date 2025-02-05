package org.firstinspires.ftc.teamcode.common.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class IntakeSetMotorState_INST extends InstantCommand {
    public IntakeSetMotorState_INST(IntakeSubsystem intakeSubsystem, IntakeSubsystem.IntakeMotorState intakeMotorState) {
        super(
                () -> intakeSubsystem.setIntakeMotorState(intakeMotorState)
        );
    }
}
