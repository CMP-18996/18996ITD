package org.firstinspires.ftc.teamcode.common.commands.specimen;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;

public class SpecimenSetGripperPosition_INST extends InstantCommand {
    public SpecimenSetGripperPosition_INST(SpecimenSubsystem specimenSubsystem, SpecimenSubsystem.SpecimenGripperState specimenGripperState) {
        super(
                () -> specimenSubsystem.setSpecimenGripperState(specimenGripperState)
        );
    }
}
