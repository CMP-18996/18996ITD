package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class ColorSensorStateCommand extends InstantCommand {
    private IntakeSubsystem intakeSubsystem;
    private IntakeSubsystem.ColorSensorState colorSensorState;

    public ColorSensorStateCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.ColorSensorState colorSensorState) {
        this.intakeSubsystem = intakeSubsystem;
        this.colorSensorState = colorSensorState;
    }

    @Override
    public void execute() {
        intakeSubsystem.updateColorSensorState(colorSensorState);
    }
}
