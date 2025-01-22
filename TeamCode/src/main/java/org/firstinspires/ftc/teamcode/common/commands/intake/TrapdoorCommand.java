package org.firstinspires.ftc.teamcode.common.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;


public class TrapdoorCommand extends InstantCommand{

    public TrapdoorCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.TrapdoorState state){
        super(
                () -> intakeSubsystem.updateTrapdoorState(state)
        );
    }
}
