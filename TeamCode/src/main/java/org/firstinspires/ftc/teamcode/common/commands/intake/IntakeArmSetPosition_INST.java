package org.firstinspires.ftc.teamcode.common.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class IntakeArmSetPosition_INST extends InstantCommand {
    public IntakeArmSetPosition_INST(IntakeSubsystem intakeSubsystem, IntakeSubsystem.IntakeArmState intakeArmState) {
        super(
                () -> intakeSubsystem.setIntakeArmState(intakeArmState)
        );
    }
}
