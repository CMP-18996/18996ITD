package org.firstinspires.ftc.teamcode.common.commands.complexCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

// Ends when color is detected

public class WaitForColorCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final IntakeSubsystem.Color color;
    private final ElapsedTime timer = new ElapsedTime();

    public WaitForColorCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.Color color) {
        this.intakeSubsystem = intakeSubsystem;
        this.color = color;
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        if(intakeSubsystem.getColorSensorStatus().equals(IntakeSubsystem.ColorSensorStatus.DISABLED)) {
            return timer.milliseconds() >= 1000;
        }
        else {
            return intakeSubsystem.getCurrentColor().equals(color);
        }
    }
}
