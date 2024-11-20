package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class ExtensionSubsystem extends SubsystemBase {
    DcMotorImpl extensionMotor;
    ExtensionState extensionState = ExtensionState.CONTRACTED;
    public static int CONTRACTED_POS = 0;
    public static int FULL_EXTENSION_POS = 400;
    public static int HALF_EXTENDED_POS = 200;
    public static double P = .01;
    public static double F = .07;
    private int targetPosition = 0;

    public enum ExtensionState {
        CONTRACTED(CONTRACTED_POS),
        FULLY_EXTENDED(FULL_EXTENSION_POS),
        HALF_EXTENDED(HALF_EXTENDED_POS),
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

    public int getPosition() {
        return extensionMotor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public int getAbsError() {
        return Math.abs(-extensionMotor.getCurrentPosition() - extensionState.position);
    }

    public void setExtensionMotorPower(double power) {
        extensionMotor.setPower(power);
    }

    @Override
    public void periodic() {
        double error = targetPosition + extensionMotor.getCurrentPosition();
        double power;
        if (getAbsError() > 15) {
            power = Range.clip(P * error + F * (error / Math.max(abs(error), 0.01)), -.8, .8);
        }
        else power = 0;
        extensionMotor.setPower(power);
    }

    public ExtensionSubsystem(HardwareMap hardwareMap) {
        extensionMotor = hardwareMap.get(DcMotorImpl.class, "extension");
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.setState(ExtensionState.CONTRACTED);
    }
}
