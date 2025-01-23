package org.firstinspires.ftc.teamcode.common.commands.specimen;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;

public class SpecimenSetArmPosition extends CommandBase {
    SpecimenSubsystem specimenSubsystem;
    SpecimenSubsystem.SpecimenArmState specimenArmState;
    public final int tolerance;

    public SpecimenSetArmPosition(SpecimenSubsystem specimenSubsystem, SpecimenSubsystem.SpecimenArmState specimenArmState) {
        this(specimenSubsystem, specimenArmState, 10);
    }

    public SpecimenSetArmPosition(SpecimenSubsystem specimenSubsystem, SpecimenSubsystem.SpecimenArmState specimenArmState, int tolerance) {
        this.specimenSubsystem = specimenSubsystem;
        this.specimenArmState = specimenArmState;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() {
        specimenSubsystem.setSpecimenArmState(specimenArmState);
    }

    public boolean isFinished() {
        return (Math.abs(specimenSubsystem.getError()) <= tolerance
                || specimenArmState.equals(SpecimenSubsystem.SpecimenArmState.MANUAL)
                || !specimenArmState.equals(specimenSubsystem.getSpecimenArmState()));
    }
}
