package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ExtensionSubsystem extends SubsystemBase {
    DcMotor extensionMotor;
    ExtensionState extensionState = ExtensionState.CONTRACTED;
    public static int CONTRACTED_POS = 0;
    public static int FULL_EXTENSION_POS = 200;
    public static int HALF_EXTENDED_POS = 100;
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

    public int getAbsError() {
        return Math.abs(extensionMotor.getCurrentPosition() - extensionState.position);
    }

    public void setExtensionMotorPower(double power) {
        extensionMotor.setPower(power);
    }

    public ExtensionSubsystem(HardwareMap hardwareMap) {
        extensionMotor = hardwareMap.get(DcMotorImpl.class, "extension");
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.setState(ExtensionState.CONTRACTED);
    }
}
