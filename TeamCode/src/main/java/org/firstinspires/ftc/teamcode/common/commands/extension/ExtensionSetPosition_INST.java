package org.firstinspires.ftc.teamcode.common.commands.extension;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;

public class ExtensionSetPosition_INST extends InstantCommand {
    public ExtensionSetPosition_INST(ExtensionSubsystem extensionSubsystem, ExtensionSubsystem.ExtensionState extensionState) {
        super(
                () -> extensionSubsystem.setExtensionState(extensionState)
        );
    }
}
