package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class ColorSensorCommand extends CommandBase {
    public ColorSensorCommand(IntakeSubsystem intakeSubsystem) {

        intakeSubsystem.updateColorState();

    }
}
