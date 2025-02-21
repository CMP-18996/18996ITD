package org.firstinspires.ftc.teamcode.common.commands.autoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetArmPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetGripperPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;

public class AutoSpecimenGrab extends SequentialCommandGroup {
    public AutoSpecimenGrab(SpecimenSubsystem specimenSubsystem) {
        addCommands(
                new SpecimenSetGripperPosition_INST(specimenSubsystem, SpecimenSubsystem.SpecimenGripperState.CLOSED),
                new WaitCommand(400),
                new SpecimenSetArmPosition_INST(specimenSubsystem, SpecimenSubsystem.SpecimenArmState.CHAMBER)
        );
    }
}
