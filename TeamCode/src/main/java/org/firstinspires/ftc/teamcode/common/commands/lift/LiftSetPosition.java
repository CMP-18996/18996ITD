package org.firstinspires.ftc.teamcode.common.commands.lift;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class LiftSetPosition extends CommandBase {
    LiftSubsystem liftSubsystem;
    LiftSubsystem.LiftState liftState;
    int tolerance;

    public LiftSetPosition(LiftSubsystem liftSubsystem, LiftSubsystem.LiftState liftState){
        this.liftState = liftState;
        this.liftSubsystem = liftSubsystem;
        this.tolerance = 10;
    }

    public LiftSetPosition(LiftSubsystem liftSubsystem, LiftSubsystem.LiftState liftState, int tolerance){
        this.liftState = liftState;
        this.liftSubsystem = liftSubsystem;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() {
        liftSubsystem.setLiftState(liftState);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(liftSubsystem.getError()) <= tolerance);
    }
}
