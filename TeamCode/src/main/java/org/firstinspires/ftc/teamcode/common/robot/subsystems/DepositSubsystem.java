package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class DepositSubsystem extends SubsystemBase {
    public static double TRANSFER_POS = 0.05;
    public static double READY_POS = 0.5;
    public static double DEPOSIT_POS = 1.0;

    private final Servo bucketServo;
    private BucketState bucketState;

    public enum BucketState {
        TRANSFER,
        READY,
        DEPOSIT;

        public double getValue() {
            switch (this) {
                case TRANSFER:
                    return TRANSFER_POS;
                case READY:
                    return READY_POS;
                case DEPOSIT:
                    return DEPOSIT_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public DepositSubsystem(HardwareMap hardwareMap) {
        bucketServo = hardwareMap.get(Servo.class, HardwareMapNames.BUCKET_SERVO);
        bucketServo.setDirection(Servo.Direction.FORWARD);
        setBucketState(BucketState.TRANSFER);
    }

    public void setBucketState(BucketState bucketState) {
        this.bucketState = bucketState;
        bucketServo.setPosition(bucketState.getValue());
    }

    public BucketState getBucketState() {
        return bucketState;
    }
}
