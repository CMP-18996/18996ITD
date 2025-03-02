package org.firstinspires.ftc.teamcode.common.commands.complexCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositTrapdoorPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeClawSetState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakePivotSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeRollerSetState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class TransferSampleCommand extends SequentialCommandGroup {
    private ExtensionSubsystem extensionSubsystem;

    public TransferSampleCommand(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem, DepositSubsystem depositSubsystem, LiftSubsystem liftSubsystem) {
        this.extensionSubsystem = extensionSubsystem;
        addCommands(
                new LiftSetPosition_INST(liftSubsystem, LiftSubsystem.LiftState.TRANSFER),

                new DepositTrapdoorPosition_INST(depositSubsystem, DepositSubsystem.DepositTrapdoorState.OPEN),
                new DepositSetPosition_INST(depositSubsystem, DepositSubsystem.BucketState.TRANSFER),

                new ExtensionSetPosition_INST(extensionSubsystem, ExtensionSubsystem.ExtensionState.TRANSFER),

                new IntakeRollerSetState_INST(intakeSubsystem, IntakeSubsystem.IntakeRollerState.HOLD),
                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.REST),
                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.REST),
                new IntakeClawSetState_INST(intakeSubsystem, IntakeSubsystem.IntakeClawState.CLOSED),
                new IntakePivotSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakePivotState.PIVOT_0),

                new ParallelRaceGroup(
                        new WaitCommand(2000),
                        new WaitUntilCommand(() -> liftSubsystem.getError() < 5 && extensionSubsystem.getError() < 5)
                ),

                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new IntakeArmSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeArmState.TRANSFER),
                                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.TRANSFER),
                                new IntakePivotSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakePivotState.PIVOT_TRANSFER),

                                new WaitCommand(500),

                                new IntakeRollerSetState_INST(intakeSubsystem, IntakeSubsystem.IntakeRollerState.DISABLED),
                                new IntakeWristSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakeWristState.BUCKET),

                                new WaitCommand(200),

                                new IntakeClawSetState_INST(intakeSubsystem, IntakeSubsystem.IntakeClawState.OPEN),
                                new IntakeRollerSetState_INST(intakeSubsystem, IntakeSubsystem.IntakeRollerState.TRANSFER),

                                new WaitCommand(100),

                                new IntakeClawSetState_INST(intakeSubsystem, IntakeSubsystem.IntakeClawState.CLOSED),

                                new WaitCommand(500),

                                new IdleIntakeCommand(intakeSubsystem),
                                new IntakePivotSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakePivotState.PIVOT_0),
                                new DepositTrapdoorPosition_INST(depositSubsystem, DepositSubsystem.DepositTrapdoorState.CLOSED),
                                new ExtensionSetPosition_INST(extensionSubsystem, ExtensionSubsystem.ExtensionState.CUSTOM)

                                //new WaitCommand(100),

                                //new LiftSetPosition_INST(liftSubsystem, LiftSubsystem.LiftState.HIGH_BUCKET),
                                //new DepositSetPosition_INST(depositSubsystem, DepositSubsystem.BucketState.DEPOSIT)
                        ),
                        new ScheduleCommand(
                                new ExtensionSetPosition_INST(extensionSubsystem, ExtensionSubsystem.ExtensionState.CUSTOM),
                                new IntakePivotSetPosition_INST(intakeSubsystem, IntakeSubsystem.IntakePivotState.PIVOT_0),
                                new IdleIntakeCommand(intakeSubsystem)
                        ),
                        () -> liftSubsystem.getError() < 5 && extensionSubsystem.getError() < 5
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        extensionSubsystem.setExtensionState(ExtensionSubsystem.ExtensionState.CUSTOM);
    }
}
