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

public class AutoDeposit extends SequentialCommandGroup {
    public AutoDeposit(LiftSubsystem liftSubsystem, DepositSubsystem depositSubsystem) {
        addCommands(
                new ParallelDeadlineGroup(
                        new WaitCommand(1000),
                        new LiftSetPosition(liftSubsystem, LiftSubsystem.HIGH_BASKET),
                        new SequentialCommandGroup(
                                new WaitCommand(400),
                                new DepositRotationCommand(depositSubsystem, DepositSubsystem.TransferRotatorState.DEPOSITING)
                        )
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(800),
                        new LiftSetPosition(liftSubsystem, LiftSubsystem.GROUND),
                        new DepositRotationCommand(depositSubsystem, DepositSubsystem.TransferRotatorState.TRANSFER_READY)
                )
        );
    }
}
