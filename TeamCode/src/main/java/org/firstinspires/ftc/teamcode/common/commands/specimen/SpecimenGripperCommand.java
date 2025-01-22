package org.firstinspires.ftc.teamcode.common.commands.specimen;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;

public class SpecimenGripperCommand extends InstantCommand {
    public SpecimenGripperCommand(SpecimenSubsystem specimenSubsystem, SpecimenSubsystem.GripperPosition gripperState) {
        super(
                () -> specimenSubsystem.setGripperState(gripperState)
        );
    }
}
