package org.firstinspires.ftc.teamcode.common.commands.autoCommands;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetRollerState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeTrapdoorSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class AutoExtend extends SequentialCommandGroup {
    public AutoExtend(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem, LiftSubsystem liftSubsystem) {
        addCommands(
                new ExtensionSetPosition(extensionSubsystem, ExtensionSubsystem.ExtensionState.TRANSFER),
                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.REST),
                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.REST),
                new IntakeSetRollerState_INST(intakeSubsystem, IntakeSubsystem.IntakeRollerState.ACTIVE),
                new LiftSetPosition_INST(liftSubsystem, LiftSubsystem.LiftState.TRANSFER),
                new IntakeTrapdoorSetPosition_INST(intakeSubsystem, IntakeSubsystem.TrapdoorState.CLOSED),
                new ParallelDeadlineGroup(
                        new WaitCommand(1000),
                        new LiftSetPosition(liftSubsystem, LiftSubsystem.LiftState.TRANSFER),
                        new ExtensionSetPosition(extensionSubsystem, ExtensionSubsystem.ExtensionState.FULLY_EXTENDED),
                        new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.PICK_UP),
                        new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.PICK_UP)
                )
        );
    }
}
