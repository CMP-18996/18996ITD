package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class MultiColorSensorCommand extends CommandBase {
    IntakeSubsystem intakeSubsystem;
    IntakeSubsystem.ColorState commandColorState;

    public MultiColorSensorCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.ColorState colorState) {
        commandColorState = colorState;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.updateColorState().equals(commandColorState)
                || intakeSubsystem.updateColorState().equals(IntakeSubsystem.ColorState.YELLOW);
    }
}
