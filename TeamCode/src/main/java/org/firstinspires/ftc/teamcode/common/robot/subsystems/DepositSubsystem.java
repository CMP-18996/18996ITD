package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class DepositSubsystem extends SubsystemBase {
    public static double TRANSFER_POS = 0.05;
    public static double DEPOSIT_POS = 0.8;

    public static double OPEN_POS = 0.0;
    public static double CLOSED_POS = 0.9;

    private final Servo bucketServo;
    private final Servo depositTrapdoorServo;
    private BucketState bucketState;
    private DepositTrapdoorState depositTrapdoorState;

    public enum BucketState {
        TRANSFER,
        DEPOSIT;

        public double getValue() {
            switch (this) {
                case TRANSFER:
                    return TRANSFER_POS;
                case DEPOSIT:
                    return DEPOSIT_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum DepositTrapdoorState {
        OPEN,
        CLOSED;

        public double getValue() {
            switch (this) {
                case OPEN:
                    return OPEN_POS;
                case CLOSED:
                    return CLOSED_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public DepositSubsystem(HardwareMap hardwareMap) {
        bucketServo = hardwareMap.get(Servo.class, HardwareMapNames.BUCKET_SERVO);
        depositTrapdoorServo = hardwareMap.get(Servo.class, HardwareMapNames.DEPOSIT_TRAPDOOR);

        bucketServo.setDirection(Servo.Direction.FORWARD);
        depositTrapdoorServo.setDirection(Servo.Direction.FORWARD);

        setBucketState(BucketState.TRANSFER);
        setDepositTrapdoorState(DepositTrapdoorState.CLOSED);
    }

    public void setBucketState(BucketState bucketState) {
        this.bucketState = bucketState;
        bucketServo.setPosition(bucketState.getValue());
    }

    public BucketState getBucketState() {
        return bucketState;
    }

    public void setDepositTrapdoorState(DepositTrapdoorState depositTrapdoorState) {
        this.depositTrapdoorState = depositTrapdoorState;
        depositTrapdoorServo.setPosition(depositTrapdoorState.getValue());
    }

    public DepositTrapdoorState getDepositTrapdoorState() {
        return depositTrapdoorState;
    }
}
