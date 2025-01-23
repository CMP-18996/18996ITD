package org.firstinspires.ftc.teamcode.common.commands.complexCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

// Ends when color is detected

public class WaitForColorCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final IntakeSubsystem.Color color;

    public WaitForColorCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.Color color) {
        this.intakeSubsystem = intakeSubsystem;
        this.color = color;
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.getCurrentColor().equals(color);
    }
}
