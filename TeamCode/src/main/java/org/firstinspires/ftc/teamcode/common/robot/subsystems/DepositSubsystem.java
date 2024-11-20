package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class DepositSubsystem extends SubsystemBase {
    // Constants
    public static double TRANSFER_ROTATOR_SERVO_MIN_ROT = 0.0;
    public static double TRANSFER_ROTATOR_SERVO_MAX_ROT = 0.7;


    // State
    private final Servo transferRotatorServo;
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
        transferRotatorServo = hardwareMap.get(Servo.class, HardwareMapNames.BUCKET_SERVO);
        this.updateTransferRotatorState(TransferRotatorState.TRANSFER_READY);
    }
}
