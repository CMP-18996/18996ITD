package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class LiftSubsystem extends SubsystemBase {
    public static double Kp = 0.01;
    public static double Ki = 0.04;
    public static double Kd = 0.0003;
    public static double Kf = 0.3;
    public static int INTEGRAL_ENABLE_POINT = 20;

    public static int TRANSFER_POS = 0;
    public static int LOW_BASKET_POS = 420;
    public static int HIGH_BASKET_POS = 850;

    public static double MAX_UP_SPEED = 1.0;
    public static double MAX_DOWN_SPEED = 0.6;

    private final DcMotorEx liftMotor;

    private LiftState liftState;

    private int lastError = 0;
    private double integralSum = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public enum LiftState {
        TRANSFER,
        LOW_BUCKET,
        HIGH_BUCKET;

        public int getValue() {
            switch (this) {
                case TRANSFER:
                    return TRANSFER_POS;
                case LOW_BUCKET:
                    return LOW_BASKET_POS;
                case HIGH_BUCKET:
                    return HIGH_BASKET_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public LiftSubsystem(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotorEx.class, HardwareMapNames.LIFT_MOTOR);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.setLiftState(LiftState.TRANSFER);
    }

    public void setLiftState(LiftState liftState) {
        this.liftState = liftState;
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    public LiftState getLiftState() {
        return liftState;
    }

    public int getError() {
        return liftState.getValue() - liftMotor.getCurrentPosition();
    }

    @Override
    public void periodic() {
        int error = getError();

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

        double power = Range.clip(P + I + D + F, -MAX_DOWN_SPEED, MAX_UP_SPEED);

        liftMotor.setPower(power);
    }
}
