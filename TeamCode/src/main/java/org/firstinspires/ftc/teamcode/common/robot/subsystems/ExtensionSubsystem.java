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
public class ExtensionSubsystem extends SubsystemBase {
    public static double Kp = 0.016;
    public static double Ki = 0.000000;
    public static double Kd = 0.0003;
    public static double Kf = 0.0;
    public static int INTEGRAL_ENABLE_POINT = 10;

    public static int TRANSFER_POS = 0;
    public static int EXTENDED_POS = 620;

    public static double MAX_EXTENSION_SPEED = 1.0;
    public static double MAX_RETRACTION_SPEED = 1.0;

    public final DcMotorEx extensionMotor;

    public ExtensionState extensionState;

    private int lastError = 0;
    private double integralSum = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public enum ExtensionState {
        TRANSFER,
        EXTENDED,
        CUSTOM,
        ZEROING,
        INSPECTION;

        public int getValue() {
            switch (this) {
                case TRANSFER:
                    return TRANSFER_POS;
                case EXTENDED:
                    return EXTENDED_POS;
                case CUSTOM:
                    return 0;
                case ZEROING:
                    return 0;
                case INSPECTION:
                    return 500;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public ExtensionSubsystem(HardwareMap hardwareMap) {
        extensionMotor = hardwareMap.get(DcMotorEx.class, HardwareMapNames.EXTENSION_MOTOR);

        extensionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.setExtensionState(ExtensionState.CUSTOM);
    }

    public void setExtensionState(ExtensionState extensionState) {
        this.extensionState = extensionState;
        if(extensionState.equals(ExtensionState.CUSTOM)) {
            setExtensionMotorPower(0);
        }
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    public ExtensionState getExtensionState() {
        return extensionState;
    }

    public int getError() {
        if(extensionState.equals(ExtensionState.CUSTOM)) {
            return 0;
        }
        else {
            return extensionState.getValue() - extensionMotor.getCurrentPosition();
        }
    }

    public void setExtensionMotorPower(double power) {
        extensionMotor.setPower(power);
    }

    public void setMaxExtensionSpeed(double maxExtensionSpeed) {
        MAX_EXTENSION_SPEED = maxExtensionSpeed;
    }

    public void setMaxRetractionSpeed(double maxRetractionSpeed) {
        MAX_RETRACTION_SPEED = maxRetractionSpeed;
    }

    public void setExtendedEncoderValue(int position) {
        EXTENDED_POS = position;
    }

    public void resetEncoder() {
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        if (extensionState == ExtensionState.ZEROING) {
            extensionMotor.setPower(0.2);
        }
        else if (!extensionState.equals(ExtensionState.CUSTOM)) {
            int error = getError();

            double P = Kp * error;

            if (Math.abs(error) > INTEGRAL_ENABLE_POINT) {
                integralSum = 0;
            } else {
                integralSum = integralSum + (error * timer.seconds());
            }

            double I = Ki * integralSum;

            double D = Kd * (error - lastError) / timer.seconds();

            double F = Kf * ((double)error / (double)Math.abs(error));

            lastError = error;
            timer.reset();

            double power = Range.clip(P + I + D + F, -MAX_RETRACTION_SPEED, MAX_EXTENSION_SPEED);

            extensionMotor.setPower(-power);
        }
    }
}
