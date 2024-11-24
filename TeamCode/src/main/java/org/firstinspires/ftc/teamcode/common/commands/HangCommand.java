package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class HangCommand extends CommandBase {
    HangSubsystem hangSubsystem;

    HangSubsystem.HangPosition state;
    public HangCommand(HangSubsystem hangSubsystem, HangSubsystem.HangPosition newState){
        state = newState;
        this.hangSubsystem = hangSubsystem;
        addRequirements(hangSubsystem);
    }

    @Override
    public void initialize() {
        hangSubsystem.updatePosition(state);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(hangSubsystem.getError()) < 10;
    }
}
