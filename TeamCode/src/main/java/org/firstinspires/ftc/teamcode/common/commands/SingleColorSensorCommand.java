package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

/**
 * Stops only after inputted color detected
 */
public class SingleColorSensorCommand extends CommandBase {
    IntakeSubsystem intakeSubsystem;
    IntakeSubsystem.ColorState commandColorState;

    public SingleColorSensorCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.ColorState colorState) {
        commandColorState = colorState;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public boolean isFinished() {
        return (intakeSubsystem.colorState.equals(commandColorState));
    }
}
