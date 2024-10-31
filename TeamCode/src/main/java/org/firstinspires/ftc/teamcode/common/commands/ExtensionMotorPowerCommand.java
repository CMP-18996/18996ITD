package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;

public class ExtensionMotorPowerCommand extends InstantCommand {
    public ExtensionMotorPowerCommand(ExtensionSubsystem extensionSubsystem, Double power) {
        super(
                () -> extensionSubsystem.setExtensionMotorPower(power)
        );
    }
}
