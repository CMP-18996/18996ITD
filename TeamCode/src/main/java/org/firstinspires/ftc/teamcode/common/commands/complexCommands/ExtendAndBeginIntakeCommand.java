package org.firstinspires.ftc.teamcode.common.commands.complexCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetRollerState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class ExtendAndBeginIntakeCommand extends SequentialCommandGroup {
    public ExtendAndBeginIntakeCommand(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.MOVING),
                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.MOVING),
                new IntakeSetRollerState_INST(intakeSubsystem, IntakeSubsystem.IntakeRollerState.ACTIVE),
                new ExtensionSetPosition(extensionSubsystem, ExtensionSubsystem.ExtensionState.FULLY_EXTENDED),
                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.PICK_UP),
                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.PICK_UP)
        );
    }
}
