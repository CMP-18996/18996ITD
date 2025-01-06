package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class LiftSetPosition extends CommandBase {
    LiftSubsystem liftSubsystem;
    LiftSubsystem.LiftState target;

    public LiftSetPosition(LiftSubsystem liftSubsystem, LiftSubsystem.LiftState targetState){
        target = targetState;
        this.liftSubsystem = liftSubsystem;
        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
        liftSubsystem.setTargetPosition(target);
    }

    @Override
    public boolean isFinished() {
        return (liftSubsystem.getAbsError() < 5);
    }
}
