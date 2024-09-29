package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DepositSubsystem extends SubsystemBase {
    // Constants
    final static double TRANSFER_ROTATOR_SERVO_MIN_ROT = 0.0;
    final static double TRANSFER_ROTATOR_SERVO_MAX_ROT = 0.0;


    // State
    private final ServoEx transferRotatorServo;
    private TransferRotatorState transferRotatorState = TransferRotatorState.TRANSFER_READY;
    public enum TransferRotatorState {
        TRANSFER_READY(TRANSFER_ROTATOR_SERVO_MIN_ROT),
        DEPOSITING(TRANSFER_ROTATOR_SERVO_MAX_ROT);
        public double val;
        TransferRotatorState(double inVal) { val = inVal; }
    }

    public void updateTransferRotatorState(TransferRotatorState setState) {
        transferRotatorState = setState;
        transferRotatorServo.setPosition(transferRotatorState.val);
    }

    public TransferRotatorState getTransferRotatorState() {
        return transferRotatorState;
    }

    public DepositSubsystem(HardwareMap hardwareMap) {
        transferRotatorServo = new SimpleServo(hardwareMap, "transferRotator", TRANSFER_ROTATOR_SERVO_MIN_ROT, TRANSFER_ROTATOR_SERVO_MAX_ROT);
    }
}
