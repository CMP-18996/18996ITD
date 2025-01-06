package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class InstantLiftCommand extends InstantCommand {
    public InstantLiftCommand(LiftSubsystem liftSubsystem, LiftSubsystem.LiftState target) {
        super(
                () -> liftSubsystem.setTargetPosition(target)
        );
    }
}
