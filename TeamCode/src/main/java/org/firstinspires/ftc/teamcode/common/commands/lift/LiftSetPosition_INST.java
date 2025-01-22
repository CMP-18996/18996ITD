package org.firstinspires.ftc.teamcode.common.commands.lift;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class LiftSetPosition_INST extends InstantCommand {
    public LiftSetPosition_INST(LiftSubsystem liftSubsystem, LiftSubsystem.LiftState liftState) {
        super(
                () -> liftSubsystem.setLiftState(liftState)
        );
    }
}
