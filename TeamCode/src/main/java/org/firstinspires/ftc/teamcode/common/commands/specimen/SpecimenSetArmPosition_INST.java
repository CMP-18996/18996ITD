package org.firstinspires.ftc.teamcode.common.commands.specimen;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;

public class SpecimenSetArmPosition_INST extends InstantCommand {
    public SpecimenSetArmPosition_INST(SpecimenSubsystem specimenSubsystem, SpecimenSubsystem.SpecimenGripperState specimenGripperState) {
        super(
                () -> specimenSubsystem.setSpecimenGripperState(specimenGripperState)
        );
    }
}
