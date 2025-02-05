package org.firstinspires.ftc.teamcode.common.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;


public class IntakeTrapdoorSetPosition_INST extends InstantCommand{

    public IntakeTrapdoorSetPosition_INST(IntakeSubsystem intakeSubsystem, IntakeSubsystem.IntakeTrapdoorState intakeTrapdoorState){
        super(
                () -> intakeSubsystem.setTrapdoorState(intakeTrapdoorState)
        );
    }
}
