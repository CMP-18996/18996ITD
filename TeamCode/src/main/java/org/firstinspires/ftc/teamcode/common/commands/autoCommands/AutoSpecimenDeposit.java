package org.firstinspires.ftc.teamcode.common.commands.autoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetArmPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetGripperPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;

public class AutoSpecimenDeposit extends SequentialCommandGroup {
    public AutoSpecimenDeposit(SpecimenSubsystem specimenSubsystem) {
        addCommands(
                new SpecimenSetGripperPosition_INST(specimenSubsystem, SpecimenSubsystem.SpecimenGripperState.OPEN),
                new WaitCommand(200),
                new SpecimenSetArmPosition_INST(specimenSubsystem, SpecimenSubsystem.SpecimenArmState.WALL)
        );
    }
}
