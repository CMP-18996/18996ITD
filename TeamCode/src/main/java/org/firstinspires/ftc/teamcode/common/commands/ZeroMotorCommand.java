package org.firstinspires.ftc.teamcode.common.commands;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class ZeroMotorCommand extends CommandBase {
    public ExtensionSubsystem extensionSubsystem;
    public LiftSubsystem liftSubsystem;
    private boolean executed = false;
    ElapsedTime timer = new ElapsedTime();

    public ZeroMotorCommand(ExtensionSubsystem extensionSubsystem, LiftSubsystem liftSubsystem) {
        this.extensionSubsystem = extensionSubsystem;
        this.liftSubsystem = liftSubsystem;
    }

    @Override
    public void execute() {
        if (!executed) {
            timer.startTime();

            extensionSubsystem.setExtensionMotorPower(-1);
            liftSubsystem.liftMotor.setPower(-1);
            executed = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.milliseconds() > 400) {
            liftSubsystem.liftMotor.setPower(0);
            liftSubsystem.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftSubsystem.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extensionSubsystem.setExtensionMotorPower(0);
            liftSubsystem.toggleLift();
            return true;
        }
        return false;
    }
}
