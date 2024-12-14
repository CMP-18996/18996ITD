package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class LiftSubsystem extends SubsystemBase {
    // Constants
    public static double Kp = 0.0175;
    public static double Ki = 0.01;
    public static double Kd = 0.001;
    public static double Kf = 0.15;

    public static int INTEGRAL_ENABLE_POINT;

    public static int GROUND = 40;
    public static int LOW_BASKET = 420;
    public static int HIGH_BASKET = 850;

    // State
    public final DcMotorImpl liftMotor;
    public int currTarget;

    private int lastError = 0;
    public double integralSum = 0;

    ElapsedTime timer = new ElapsedTime();

    public double telemetryPower;

    public void setTargetPosition(int targetPosition) {
        currTarget = targetPosition;
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    public int getCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }

    public int getCurrTarget() {
        return currTarget;
    }

    @Override
    public void periodic() {
        int error = currTarget - liftMotor.getCurrentPosition();

        double P = Kp * error;

        if (Math.abs(error) > INTEGRAL_ENABLE_POINT) {
            integralSum = 0;
        }
        else {
            integralSum = integralSum + (error * timer.seconds());
        }

        double I = Ki * integralSum;

        double D = Kd * (error - lastError) / timer.seconds();

        double F = Kf;

        lastError = error;
        timer.reset();

        double power = Range.clip(P + I + D + F, -0.7, 1.0);
        telemetryPower = power;

        liftMotor.setPower(power);
    }


    public boolean motorWorking() {
        return liftMotor.isBusy();
    }


    public int getAbsError() {
        return Math.abs(currTarget - liftMotor.getCurrentPosition());
    }

    public int getError() {
        return currTarget - liftMotor.getCurrentPosition();
    }

    public LiftSubsystem(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotorImpl.class, HardwareMapNames.LIFT_MOTOR);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        currTarget = LiftSubsystem.GROUND;
    }
}
