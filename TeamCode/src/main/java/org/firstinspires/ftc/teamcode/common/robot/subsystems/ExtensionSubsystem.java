package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ExtensionSubsystem extends SubsystemBase {
    DcMotor extensionMotor;
    ExtensionState extensionState = ExtensionState.CONTRACTED;

    public enum ExtensionState {
        CONTRACTED(0),
        FULLY_EXTENDED(100),
        HALF_EXTENDED(200),
        CUSTOM(100);
        public int position;
        ExtensionState(int value) { position = value; }
    }

    public void setState(ExtensionState newExtensionState) {
        if (ExtensionState.CUSTOM.equals(newExtensionState)) {
            if (!extensionState.equals(ExtensionState.CUSTOM)) {
                extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            extensionState = newExtensionState;
        }
        else {
            if (extensionState.equals(ExtensionState.CUSTOM)) {
                extensionMotor.setTargetPosition(extensionState.position);
                extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            extensionState = newExtensionState;
//            extensionMotor.setTargetPosition(extensionState.position);
        }
    }

    public ExtensionState getState() {
        return extensionState;
    }

    public void setPosition(int position) {
        this.setState(ExtensionState.CUSTOM);
        extensionMotor.setTargetPosition(position);
        extensionMotor.setPower(.6);
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
        extensionMotor.setTargetPosition(0);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setTargetPosition(0);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
