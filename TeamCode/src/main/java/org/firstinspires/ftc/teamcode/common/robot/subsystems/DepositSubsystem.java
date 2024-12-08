package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class DepositSubsystem extends SubsystemBase {
    // Constants
    public static double TRANSFER_ROTATOR_TRANSFER = 0.0;
    public static double TRANSFER_ROTATOR_DEPOSIT = 1.0;
    public static double TRANSFER_ROTATOR_READY = 0.5;

    // State
    private final Servo transferRotatorServo;
    private TransferRotatorState transferRotatorState = TransferRotatorState.TRANSFER_READY;
    public enum TransferRotatorState {
        TRANSFER_READY(TRANSFER_ROTATOR_TRANSFER),
        READY_TO_DEPOSIT(TRANSFER_ROTATOR_READY),
        DEPOSITING(TRANSFER_ROTATOR_DEPOSIT);

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
        transferRotatorServo = hardwareMap.get(Servo.class, HardwareMapNames.BUCKET_SERVO);
        transferRotatorServo.setDirection(Servo.Direction.FORWARD);
        this.updateTransferRotatorState(TransferRotatorState.TRANSFER_READY);
    }
}
