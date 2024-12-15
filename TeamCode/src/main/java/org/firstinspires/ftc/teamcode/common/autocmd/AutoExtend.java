package org.firstinspires.ftc.teamcode.common.autocmd;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commands.DepositRotationCommand;
import org.firstinspires.ftc.teamcode.common.commands.ExtendCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class AutoExtend extends SequentialCommandGroup {
    public AutoExtend(ExtensionSubsystem extensionSubsystem, IntakeSubsystem intakeSubsystem, LiftSubsystem liftSubsystem) {
        addCommands(
                new ExtendCommand(extensionSubsystem, ExtensionSubsystem.ExtensionState.CONTRACTED),
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.PICKING_UP),
                new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.ACTIVE),
                new ParallelDeadlineGroup(
                        new WaitCommand(1400),
                        new LiftSetPosition(liftSubsystem, LiftSubsystem.GROUND),
                        new ExtendCommand(extensionSubsystem, ExtensionSubsystem.ExtensionState.FULLY_EXTENDED)
                )
        );
    }
}
