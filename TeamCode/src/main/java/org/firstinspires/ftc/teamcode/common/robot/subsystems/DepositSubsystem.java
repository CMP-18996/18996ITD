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
    public static double TOP_OPEN_POS = 0.0;
    public static double BOTTOM_OPEN_POS = 0.0;
    public static double CLOSED_POS = 0.0;

    private final Servo bucketServo;
    private final Servo depositTrapdoorServo;
    private BucketState bucketState;
    private DepositTrapdoorState depositTrapdoorState;

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

    public enum DepositTrapdoorState {
        TOP_OPEN(TOP_OPEN_POS),
        BOTTOM_OPEN(BOTTOM_OPEN_POS),
        CLOSED(CLOSED_POS);

        public double position;
        DepositTrapdoorState(double position) {
            this.position = position;
        }
    }

    public DepositSubsystem(HardwareMap hardwareMap) {
        bucketServo = hardwareMap.get(Servo.class, HardwareMapNames.BUCKET_SERVO);
        bucketServo.setDirection(Servo.Direction.FORWARD);
        setBucketState(BucketState.TRANSFER);
        depositTrapdoorServo = hardwareMap.get(Servo.class, HardwareMapNames.DEPOSIT_TRAPDOOR);
        depositTrapdoorServo.setDirection(Servo.Direction.FORWARD);
        setDepositTrapdoor(DepositTrapdoorState.TOP_OPEN);
    }

    public void setBucketState(BucketState bucketState) {
        this.bucketState = bucketState;
        bucketServo.setPosition(bucketState.getValue());
    }

    public void setDepositTrapdoor(DepositTrapdoorState state) {
        this.depositTrapdoorState = state;
        depositTrapdoorServo.setPosition(depositTrapdoorState.position);
    }

    public BucketState getBucketState() {
        return bucketState;
    }
}
