package org.firstinspires.ftc.teamcode.common.commands.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class ZeroLift extends CommandBase {
    private LiftSubsystem liftSubsystem;
    private ElapsedTime timer = new ElapsedTime();

    public ZeroLift(LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;
    }

    @Override
    public void initialize() {
        liftSubsystem.setLiftState(LiftSubsystem.LiftState.ZEROING);
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return (timer.milliseconds() > 100);
    }

    @Override
    public void end(boolean interrupted) {
        liftSubsystem.resetEncoder();
        liftSubsystem.setLiftState(LiftSubsystem.LiftState.TRANSFER);
    }
}
