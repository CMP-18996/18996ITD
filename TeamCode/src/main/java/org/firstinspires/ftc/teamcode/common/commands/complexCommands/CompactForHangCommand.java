package org.firstinspires.ftc.teamcode.common.commands.complexCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositTrapdoorPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetMotorState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetArmPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetGripperPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;

public class CompactForHangCommand extends SequentialCommandGroup {
    public CompactForHangCommand(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem, DepositSubsystem depositSubsystem, LiftSubsystem liftSubsystem, SpecimenSubsystem specimenSubsystem) {
        addCommands(
                new LiftSetPosition_INST(liftSubsystem, LiftSubsystem.LiftState.TRANSFER),

                new DepositSetPosition_INST(depositSubsystem, DepositSubsystem.BucketState.TRANSFER),
                new DepositTrapdoorPosition_INST(depositSubsystem, DepositSubsystem.DepositTrapdoorState.TOP_OPEN),

                new ExtensionSetPosition_INST(extensionSubsystem, ExtensionSubsystem.ExtensionState.TRANSFER),

                new SpecimenSetArmPosition_INST(specimenSubsystem, SpecimenSubsystem.SpecimenArmState.WALL),
                new SpecimenSetGripperPosition_INST(specimenSubsystem, SpecimenSubsystem.SpecimenGripperState.OPEN),

                new IntakeSetMotorState_INST(intakeSubsystem, IntakeSubsystem.IntakeMotorState.DISABLED),

                new WaitCommand(500),
                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.TRANSFER),
                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.TRANSFER)
        );
    }
}
