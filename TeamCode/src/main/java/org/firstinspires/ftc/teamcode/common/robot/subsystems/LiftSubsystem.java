package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class LiftSubsystem extends SubsystemBase {
    // Constants
    public static double P = .03;
    public static double F = .1;
    public static int GROUND = 0;
    public static int LOW_RUNG = 250;
    public static int HIGH_RUNG = 650;
    public static int LOW_BASKET = 450;
    public static int HIGH_BASKET = 850;

    // State
    private final DcMotorImpl liftMotor;
    private int currTarget = 0;

    public void setTargetPosition(int targetPosition) {
        currTarget = targetPosition;
    }

    public int getCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }
    public int getCurrTarget() {
        return currTarget;
    }

    @Override
    public void periodic() {
        double error = currTarget - liftMotor.getCurrentPosition();
        double power = Range.clip(P * error + F * (error / Math.max(abs(error), 0.01)), -.6, 1);
        liftMotor.setPower(power);
    }

    public boolean motorWorking() {
        return liftMotor.isBusy();
    }

    public int getAbsError() {
        return Math.abs(currTarget - liftMotor.getCurrentPosition());
    }

    public LiftSubsystem(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotorImpl.class, "lift"); // port 1 as of 11/18
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
