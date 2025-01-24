package org.firstinspires.ftc.teamcode.common.commands.lift;

import static android.os.SystemClock.sleep;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class LiftZeroEncoder extends CommandBase {
    private final LiftSubsystem liftSubsystem;
    ElapsedTime timer = new ElapsedTime();

    public LiftZeroEncoder(LiftSubsystem liftSubsystem){
        this.liftSubsystem = liftSubsystem;
    }

    @Override
    public void initialize() {
        timer.reset();
        liftSubsystem.setLiftState(LiftSubsystem.LiftState.ZEROING);
        liftSubsystem.setLiftMotorPower(-1.0);
    }

    @Override
    public boolean isFinished() {
        if(timer.milliseconds() >= 200) {
            liftSubsystem.setLiftMotorPower(0);
            liftSubsystem.setLiftState(LiftSubsystem.LiftState.TRANSFER);
            return true;
        }
        else {
            return false;
        }
    }
}
