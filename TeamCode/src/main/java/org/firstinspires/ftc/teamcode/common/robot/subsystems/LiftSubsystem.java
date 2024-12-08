package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class LiftSubsystem extends SubsystemBase {
    // Constants
    public static double P = 0.01;
    public static double F = 0.25;
    public static int GROUND = 30;
    public static int LOW_BASKET = 420;
    public static int HIGH_BASKET = 860;

    // State
    private final DcMotorImpl liftMotor;
    private int currTarget = 30;

    public double powerTELE;

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
        double power;

        /*
        if (getAbsError() > 15) {
            power = Range.clip(P * error + F * (error / Math.max(abs(error), 0.01)), -.6, 1);
        }
        else power = 0;
        */
        power = Range.clip(P * error + F, -0.6, 1.0);
        powerTELE = power;

        liftMotor.setPower(power);
    }

    public boolean motorWorking() {
        return liftMotor.isBusy();
    }

    public int getAbsError() {
        return Math.abs(currTarget - liftMotor.getCurrentPosition());
    }

    public LiftSubsystem(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotorImpl.class, HardwareMapNames.LIFT_MOTOR); // port 1 as of 11/18
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
