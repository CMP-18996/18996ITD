package org.firstinspires.ftc.teamcode.common.commands.autoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeRollerSetState_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class AutoTossToHumanPlayer extends SequentialCommandGroup {
    public AutoTossToHumanPlayer(IntakeSubsystem intakeSubsystem) {
        addCommands(
                new IntakeRollerSetState_INST(intakeSubsystem, IntakeSubsystem.IntakeRollerState.REVERSING),
                new WaitCommand(200),
                new IntakeRollerSetState_INST(intakeSubsystem, IntakeSubsystem.IntakeRollerState.ACTIVE)
        );
    }
}
