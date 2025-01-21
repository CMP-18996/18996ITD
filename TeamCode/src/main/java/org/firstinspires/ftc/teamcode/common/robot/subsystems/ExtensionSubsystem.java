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
public class ExtensionSubsystem extends SubsystemBase {
    DcMotorImpl extensionMotor;
    ExtensionState extensionState;
    public static int CONTRACTED_POS = 0;
    public static int FULL_EXTENSION_POS = 500;
    public static int TRANSFER_POS = 150;
    public static int INTEGRAL_ENABLE_POINT = 10;
    public static double KP = .007;
    public static double KI = .000001;
    public static double KD = .00001;
    public static double KF = .35;
    private double error = 0;
    private double lastError = 0;
    private double integralSum = 0;

    private int targetPosition = 0;
    public static double maxPower = .8;
    ElapsedTime timer = new ElapsedTime();

    public double telemetryPower;

    public enum ExtensionState {
        CONTRACTED(CONTRACTED_POS),
        FULLY_EXTENDED(FULL_EXTENSION_POS),
        CUSTOM(100);
        public int position;
        ExtensionState(int value) { position = value; }
    }

    public void setState(ExtensionState newExtensionState) {
        extensionState = newExtensionState;
        if (!extensionState.equals(ExtensionState.CUSTOM)) targetPosition = extensionState.position;
    }

    public ExtensionState getState() {
        return extensionState;
    }

    public void setTargetPosition(int position) {
        extensionState = ExtensionState.CUSTOM;
        targetPosition = position;
    }

    public void setPower(double power) {
        extensionMotor.setPower(power);
    }

    public int getPosition() {
        return extensionMotor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public double getAbsError() {
        return Math.abs(extensionMotor.getCurrentPosition() - extensionState.position);
    }

    public double getError() {
        return extensionMotor.getCurrentPosition() - extensionState.position;
    }
//    @Deprecated
    public void setExtensionMotorPower(double power) {
        extensionMotor.setPower(power);
    }

    @Override
    public void periodic() {
        if (!extensionState.equals(ExtensionState.CUSTOM)) {
            double I, D, F, P, power;
            error = targetPosition - extensionMotor.getCurrentPosition();
            double feedForwardPower = 0;
            if (getAbsError() > INTEGRAL_ENABLE_POINT) {
                integralSum = 0;
            }
            else {
                integralSum = integralSum + (error * timer.seconds());
            }
            P = KP * error;
            I = KI * integralSum;
            D = KD * (error - lastError)/timer.seconds();
            F = KF;
            lastError = error;
            timer.reset();
            power = Range.clip(P * error + feedForwardPower, -maxPower, maxPower);

//          if (getAbsError() <= 155) power *= .45;

            telemetryPower = -power;
            extensionMotor.setPower(-power);
        }
    }

    public ExtensionSubsystem(HardwareMap hardwareMap) {
        extensionMotor = hardwareMap.get(DcMotorImpl.class, HardwareMapNames.EXTENSION_MOTOR);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extensionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.setState(ExtensionState.CUSTOM);
    }

    public ExtensionSubsystem(HardwareMap hardwareMap, boolean TeleOp) {
        extensionMotor = hardwareMap.get(DcMotorImpl.class, HardwareMapNames.EXTENSION_MOTOR);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extensionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.setState(ExtensionState.CUSTOM);
    }

    public void setMaxPower(double m) {
        maxPower = m;
    }
}
