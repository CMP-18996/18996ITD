package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class DepositSubsystem extends SubsystemBase {
    public static double BUCKET_TRANSFER = 0.00;
    public static double BUCKET_READY = 0.5;
    public static double BUCKET_DEPOSIT = 1.0;

    private final Servo bucketServo;
    private BucketState bucketState;

    public enum BucketState {
        TRANSFER,
        READY,
        DEPOSIT;

        public double getValue() {
            switch (this) {
                case TRANSFER:
                    return BUCKET_TRANSFER;
                case READY:
                    return BUCKET_READY;
                case DEPOSIT:
                    return BUCKET_DEPOSIT;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public DepositSubsystem(HardwareMap hardwareMap) {
        bucketServo = hardwareMap.get(Servo.class, HardwareMapNames.BUCKET_SERVO);
        bucketServo.setDirection(Servo.Direction.FORWARD);
    }

    public void setBucketState(BucketState bucketState) {
        this.bucketState = bucketState;
        bucketServo.setPosition(bucketState.getValue());
    }

    public BucketState getBucketState() {
        return bucketState;
    }
}
