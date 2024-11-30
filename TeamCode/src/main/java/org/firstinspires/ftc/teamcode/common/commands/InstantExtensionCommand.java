package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;

public class InstantExtensionCommand extends InstantCommand {
    public InstantExtensionCommand(ExtensionSubsystem extensionSubsystem, ExtensionSubsystem.ExtensionState setExtensionState) {
        super(
                () -> extensionSubsystem.setState(setExtensionState)
        );
    }
}
