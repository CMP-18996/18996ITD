package org.firstinspires.ftc.teamcode.common.commands.hang;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.HangSubsystem;

public class HangCommand extends InstantCommand {
    public HangCommand(HangSubsystem hangSubsystem, HangSubsystem.HangState hangState) {
        super(
                () -> hangSubsystem.setState(hangState)
        );
    }
}

