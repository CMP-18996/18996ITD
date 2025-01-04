package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class DepositSubsystem extends SubsystemBase {
    // Constants
    public static double TRANSFER_ROTATOR_TRANSFER = 0.025;
    public static double TRANSFER_ROTATOR_DEPOSIT = 1.0;
    public static double TRANSFER_ROTATOR_READY = 0.5;
    public static double TRANSFER_ROTATOR_INTERMEDIATE = .35;

    // State
    private final Servo transferRotatorServo;
    final private Servo transferClawServo;
    private ClawState clawState = ClawState.REST;

    private TransferRotatorState transferRotatorState = TransferRotatorState.TRANSFER_READY;
    public enum TransferRotatorState {
        TRANSFER_READY(TRANSFER_ROTATOR_TRANSFER),
        READY_TO_DEPOSIT(TRANSFER_ROTATOR_READY),
        INTERMEDIATE(TRANSFER_ROTATOR_INTERMEDIATE),
        DEPOSITING(TRANSFER_ROTATOR_DEPOSIT);

        public double val;
        TransferRotatorState(double inVal) { val = inVal; }
    }
    public enum ClawState {
        REST(0.0),
        CLOSED(0.25),
        OPEN(0.50);
        public double val;
        ClawState(double inVal) { val = inVal; }
    }

    public void updateTransferRotatorState(TransferRotatorState setState) {
        transferRotatorState = setState;
        transferRotatorServo.setPosition(transferRotatorState.val);
    }
    public void updateTransferClawState(ClawState setState) {
        clawState = setState;
        transferClawServo.setPosition(clawState.val);
    }

    public TransferRotatorState getTransferRotatorState() {
        return transferRotatorState;
    }

    public DepositSubsystem(HardwareMap hardwareMap) {
        transferRotatorServo = hardwareMap.get(Servo.class, HardwareMapNames.BUCKET_SERVO);
        transferRotatorServo.setDirection(Servo.Direction.FORWARD);
        this.updateTransferRotatorState(TransferRotatorState.TRANSFER_READY);
        transferClawServo = hardwareMap.get(Servo.class, HardwareMapNames.CLAW_SERVO);
        transferClawServo.setDirection(Servo.Direction.FORWARD);
        this.updateTransferRotatorState(TransferRotatorState.TRANSFER_READY);
    }
}
