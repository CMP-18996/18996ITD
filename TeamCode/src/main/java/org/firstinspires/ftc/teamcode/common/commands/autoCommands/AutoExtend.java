package org.firstinspires.ftc.teamcode.common.commands.autoCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeRollerSetState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class AutoExtend extends SequentialCommandGroup {
    public AutoExtend(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                new InstantCommand(() -> extensionSubsystem.setMaxExtensionSpeed(0.8)),

                new ExtensionSetPosition_INST(extensionSubsystem, ExtensionSubsystem.ExtensionState.EXTENDED),

                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.PICK_UP),
                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.PICK_UP),
                new IntakeRollerSetState_INST(intakeSubsystem, IntakeSubsystem.IntakeRollerState.ACTIVE)
        );
    }
}
