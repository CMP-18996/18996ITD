package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class ColorSensorCommand extends CommandBase {
    IntakeSubsystem intakeSubsystem;
    IntakeSubsystem.ColorState commandColorState;

    public ColorSensorCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.ColorState colorState) {
        commandColorState = colorState;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public boolean isFinished() {
        return (intakeSubsystem.updateColorState().equals(commandColorState));
    }
}
