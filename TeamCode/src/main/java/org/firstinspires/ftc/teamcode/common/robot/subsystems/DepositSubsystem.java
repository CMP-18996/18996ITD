package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class DepositSubsystem extends SubsystemBase {
    public static double HUMAN_PLAYER_DEPOSIT_POS = 0.00;
    public static double TRANSFER_POS = 0.3;
    public static double DEPOSIT_POS = 1.0;

    public static double TOP_OPEN_POS = 0.9;
    public static double BOTTOM_OPEN_POS = 0.3;
    public static double CLOSED_POS = 0.5;

    private final Servo bucketServo;
    private final Servo depositTrapdoorServo;
    private BucketState bucketState;
    private DepositTrapdoorState depositTrapdoorState;

    public enum BucketState {
        HUMAN_PLAYER_DEPOSIT,
        TRANSFER,
        DEPOSIT;

        public double getValue() {
            switch (this) {
                case HUMAN_PLAYER_DEPOSIT:
                    return HUMAN_PLAYER_DEPOSIT_POS;
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
        TOP_OPEN,
        BOTTOM_OPEN,
        CLOSED;

        public double getValue() {
            switch (this) {
                case TOP_OPEN:
                    return TOP_OPEN_POS;
                case BOTTOM_OPEN:
                    return BOTTOM_OPEN_POS;
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
