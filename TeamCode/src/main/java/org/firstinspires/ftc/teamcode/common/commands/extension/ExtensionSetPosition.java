package org.firstinspires.ftc.teamcode.common.commands.extension;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem.ExtensionState;

public class ExtensionSetPosition extends CommandBase {
    private final ExtensionState extensionState;
    private final ExtensionSubsystem extensionSubsystem;
    private final int tolerance;

    public ExtensionSetPosition(ExtensionSubsystem extensionSubsystem, ExtensionState extensionState) {
        this(extensionSubsystem, extensionState, 10);
    }

    public ExtensionSetPosition(ExtensionSubsystem extensionSubsystem, ExtensionState extensionState, int tolerance) {
        this.extensionSubsystem = extensionSubsystem;
        this.extensionState = extensionState;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() {
        extensionSubsystem.setExtensionState(extensionState);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(extensionSubsystem.getError()) <= tolerance
                || extensionState.equals(ExtensionState.CUSTOM)
                || !extensionState.equals(extensionSubsystem.getExtensionState()));
    }
}
