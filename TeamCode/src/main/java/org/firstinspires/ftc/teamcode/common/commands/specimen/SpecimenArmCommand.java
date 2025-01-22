package org.firstinspires.ftc.teamcode.common.commands.specimen;

import com.arcrobotics.ftclib.command.CommandBase;

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
//        specimenSubsystem.setSpecimenPosition(specimenPosition);
    }

    @Override
    public void execute() {
        specimenSubsystem.setSpecimenPosition(specimenPosition);
    }


    public boolean isFinished() {
//        return Math.abs(specimenSubsystem.getError()) < 5 || specimenSubsystem.getSpecimenState() != specimenPosition;
        return true;
    }
}
