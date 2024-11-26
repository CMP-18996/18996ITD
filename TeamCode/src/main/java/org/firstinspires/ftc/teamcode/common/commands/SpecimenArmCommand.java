package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;

public class SpecimenArmCommand extends CommandBase {
    SpecimenSubsystem specimenSubsystem;
    SpecimenSubsystem.SpecimenPosition specimenPosition;
    public SpecimenArmCommand(SpecimenSubsystem specimenSubsystem, SpecimenSubsystem.SpecimenPosition specimenPosition) {
        this.specimenSubsystem = specimenSubsystem;
        this.specimenPosition = specimenPosition;
    }

    @Override
    public void initialize() {
        specimenSubsystem.setSpecimenPosition(specimenPosition);
    }

    public boolean isFinished() {
        return true;
    }
}
