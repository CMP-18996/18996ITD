package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem.ExtensionState;

public class ExtendCommand extends CommandBase {
    private final ExtensionState extensionState;
    private final ExtensionSubsystem extensionSubsystem;

    public ExtendCommand(ExtensionSubsystem subsystem, ExtensionState setExtensionState) {
        extensionSubsystem = subsystem;
        extensionState = setExtensionState;
    }

    @Override
    public void initialize() {
        extensionSubsystem.setState(extensionState);
    }

    @Override
    public boolean isFinished() {
        if (extensionSubsystem.getAbsError() < 10
                || extensionState.equals(ExtensionState.CUSTOM)
                || !extensionState.equals(extensionSubsystem.getState())) {
            extensionSubsystem.setExtensionMotorPower(0);
            return true;
        }
        return false;
    }
}
