package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;


public class LiftTrapdoorCommand extends InstantCommand{

    public LiftTrapdoorCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.TrapdoorState state){
        super(
                () -> intakeSubsystem.updateTrapdoorState(state)
        );
    }
}
