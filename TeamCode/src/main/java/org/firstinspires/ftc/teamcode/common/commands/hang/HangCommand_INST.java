package org.firstinspires.ftc.teamcode.common.commands.hang;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.HangSubsystem;

public class HangCommand_INST extends InstantCommand {
    public HangCommand_INST(HangSubsystem hangSubsystem, HangSubsystem.HangState hangState) {
        super(
                () -> hangSubsystem.setHangState(hangState)
        );
    }
}

