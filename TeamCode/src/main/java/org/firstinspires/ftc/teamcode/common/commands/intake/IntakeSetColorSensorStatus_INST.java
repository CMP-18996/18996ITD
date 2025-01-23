package org.firstinspires.ftc.teamcode.common.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class IntakeSetColorSensorStatus_INST extends InstantCommand {
    public IntakeSetColorSensorStatus_INST(IntakeSubsystem intakeSubsystem, IntakeSubsystem.ColorSensorStatus colorSensorStatus) {
        super(
                () -> intakeSubsystem.setColorSensorStatus(colorSensorStatus)
        );
    }
}
