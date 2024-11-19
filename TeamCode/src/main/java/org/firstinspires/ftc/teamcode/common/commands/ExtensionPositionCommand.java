package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;

public class ExtensionPositionCommand extends InstantCommand {
    public ExtensionPositionCommand(ExtensionSubsystem extensionSubsystem, int position) {
        super(
                () -> extensionSubsystem.setTargetPosition(position)
        );
    }
}
