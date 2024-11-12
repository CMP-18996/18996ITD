package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class LiftSubsystem extends SubsystemBase {
    // Constants
    private double P = .0001;
    public static int GROUND = 0;
    public static int LOW_RUNG = 100;
    public static int HIGH_RUNG = 200;
    public static int LOW_BASKET = 150;
    public static int HIGH_BASKET = 300;

    // State
    private final DcMotorImpl liftMotor;
    private int currTarget = 0;
    private int currPosition;

    public void setTargetPosition(int targetPosition) {
        currTarget = targetPosition;
    }

    public int getCurrentPosition() {
        return currPosition;
    }

    @Override
    public void periodic() {
        currPosition = liftMotor.getCurrentPosition();
        liftMotor.setTargetPosition(currTarget);
        liftMotor.setPower(P * Math.abs(currPosition - currTarget)); // TODO: implement proportional controller
    }

    public boolean motorWorking() {
        return liftMotor.isBusy();
    }
    public LiftSubsystem(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotorImpl.class, "lift");
        liftMotor.setTargetPosition(GROUND);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotor.setTargetPosition(GROUND);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        currPosition = liftMotor.getCurrentPosition();
        this.setTargetPosition(GROUND);
    }
}
