package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class HangCommand extends InstantCommand {
    public HangCommand(HangSubsystem hangSubsystem, HangSubsystem.HangPosition hangPosition) {
        super(
                () -> hangSubsystem.updatePosition(hangPosition)
        );
    }
}
//matthew wants this commented out just in case we change back
/*public class HangCommand extends CommandBase {
    HangSubsystem hangSubsystem;

    HangSubsystem.HangPosition state;
    public HangCommand(HangSubsystem hangSubsystem, HangSubsystem.HangPosition newState){
        state = newState;
        this.hangSubsystem = hangSubsystem;
        addRequirements(hangSubsystem);
    }

    @Override
    public void initialize() {
        hangSubsystem.updatePosition(state);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(hangSubsystem.getError()) < 10;
    }
}*/
