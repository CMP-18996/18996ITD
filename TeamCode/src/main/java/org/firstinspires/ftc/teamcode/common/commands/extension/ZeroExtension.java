package org.firstinspires.ftc.teamcode.common.commands.extension;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;

public class ZeroExtension extends SequentialCommandGroup {
    public ZeroExtension(ExtensionSubsystem extensionSubsystem) {
        addCommands(
                new SequentialCommandGroup(

                        //new InstantCommand(() -> extensionSubsystem.setExtensionState(ExtensionSubsystem.ExtensionState.ZEROING)),
                        new ExtensionSetPosition_INST(extensionSubsystem, ExtensionSubsystem.ExtensionState.ZEROING),
                        new WaitCommand(100),
                        new InstantCommand(extensionSubsystem::resetEncoder),
                        new ExtensionSetPosition_INST(extensionSubsystem, ExtensionSubsystem.ExtensionState.TRANSFER)
                )
        );
    }
}
