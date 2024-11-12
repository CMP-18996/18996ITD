package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class SampleColorCommand extends CommandBase {
    private IntakeSubsystem intake;
    private IntakeSubsystem.ColorState color;
    public SampleColorCommand(IntakeSubsystem intake, IntakeSubsystem.ColorState color) {
        this.intake = intake;
        this.color = color;
    }

    public void initialize() {
        CommandScheduler.getInstance().schedule(
                new IntakeCommand(intake, IntakeSubsystem.IntakingState.ACTIVE)
        );
    }

    @Override
    public boolean isFinished() {
        IntakeSubsystem.ColorState currentColor = intake.updateColorState();
        if (currentColor == color || currentColor == IntakeSubsystem.ColorState.YELLOW) {
            CommandScheduler.getInstance().schedule(
                    new IntakeCommand(intake, IntakeSubsystem.IntakingState.DISABLED)
            );
            return true;
        }
        else {
            return false;
        }
    }
}
