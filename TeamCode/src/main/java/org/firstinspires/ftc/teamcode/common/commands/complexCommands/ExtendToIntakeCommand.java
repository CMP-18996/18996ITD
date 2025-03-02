package org.firstinspires.ftc.teamcode.common.commands.complexCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeClawSetState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeRollerSetState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class ExtendToIntakeCommand extends SequentialCommandGroup {
    public ExtendToIntakeCommand(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                new ExtensionSetPosition_INST(extensionSubsystem, ExtensionSubsystem.ExtensionState.EXTENDED),

                new RestIntakeCommand(intakeSubsystem)
        );
    }
}
