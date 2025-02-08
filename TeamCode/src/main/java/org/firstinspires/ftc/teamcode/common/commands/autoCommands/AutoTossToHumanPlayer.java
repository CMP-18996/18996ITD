package org.firstinspires.ftc.teamcode.common.commands.autoCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.WeakReferenceSet;

import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetMotorState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

public class AutoTossToHumanPlayer extends SequentialCommandGroup {
    public AutoTossToHumanPlayer(IntakeSubsystem intakeSubsystem) {
        addCommands(
                new IntakeSetMotorState_INST(intakeSubsystem, IntakeSubsystem.IntakeMotorState.REVERSING),
                new WaitCommand(200),
                new IntakeSetMotorState_INST(intakeSubsystem, IntakeSubsystem.IntakeMotorState.ACTIVE)
        );
    }
}
