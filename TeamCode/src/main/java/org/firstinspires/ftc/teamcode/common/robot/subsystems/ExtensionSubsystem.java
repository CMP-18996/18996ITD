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
public class ExtensionSubsystem extends SubsystemBase {
    DcMotorImpl extensionMotor;
    ExtensionState extensionState;
    public static int CONTRACTED_POS = 0;
    public static int FULL_EXTENSION_POS = 500;
    public static int TRANSFER_POS = 150;
    public static double P = .008;
    public static double F = .30;
    private int targetPosition = 0;
    public static double maxPower = .8;

    public double error;

    public double telemetryPower;

    public enum ExtensionState {
        CONTRACTED(CONTRACTED_POS),
        FULLY_EXTENDED(FULL_EXTENSION_POS), 
        TRANSFER(TRANSFER_POS),
        ENCODER_ZERO(0),
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

    public int getAbsError() {
        return Math.abs(extensionMotor.getCurrentPosition() - extensionState.position);
    }

//    @Deprecated
    public void setExtensionMotorPower(double power) {
        extensionMotor.setPower(power);
    }

    @Override
    public void periodic() {
        if (!extensionState.equals(ExtensionState.CUSTOM)) {
            error = targetPosition - extensionMotor.getCurrentPosition();
            double power;

            double feedForwardPower = 0;
            if (getAbsError() > 10) {
                feedForwardPower = F * Math.signum(error);
            }

            power = Range.clip(P * error + feedForwardPower, -maxPower, maxPower);

            if (getAbsError() <= 155) power *= .45;

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
