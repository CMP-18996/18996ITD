package org.firstinspires.ftc.teamcode.common.commands.autoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class AutoExtendRetractTransfer extends SequentialCommandGroup {
    public AutoExtendRetractTransfer(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem, LiftSubsystem liftSubsystem, DepositSubsystem depositSubsystem) {
        addCommands(
                new AutoExtend(extensionSubsystem, intakeSubsystem, liftSubsystem),
                new AutoRetractTransfer(extensionSubsystem, intakeSubsystem, depositSubsystem, liftSubsystem)
        );
    }
}
