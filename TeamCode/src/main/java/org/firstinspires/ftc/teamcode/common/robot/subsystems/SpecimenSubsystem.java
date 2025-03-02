package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class SpecimenSubsystem extends SubsystemBase {
    public static double ARM_CHAMBER_POS = 0.686;
    public static double ARM_WALL_POS = 0.3525;
    public static double ARM_REST_POS = 0.2625;

    public static double WRIST_CHAMBER_POS = 1.0;
    public static double WRIST_WALL_POS = 0.3;
    public static double WRIST_REST_POS = 0.0;

    public static double GRIPPER_OPEN_POS = 0.65;
    public static double GRIPPER_CLOSED_POS = 0.1;

    private double armValue;
    private double wristValue;

    private final ServoImplEx wristServo;
    private final ServoImplEx gripperServo;
    private final ServoImplEx armServo;

    private SpecimenArmState specimenArmState;
    private SpecimenGripperState specimenGripperState;

    public enum SpecimenArmState {
        CHAMBER,
        WALL,
        MANUAL;

        public double getArmValue() {
            switch (this) {
                case CHAMBER:
                    return ARM_CHAMBER_POS;
                case WALL:
                    return ARM_WALL_POS;
                case MANUAL:
                    return ARM_REST_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }

        public double getWristValue() {
            switch (this) {
                case CHAMBER:
                    return WRIST_CHAMBER_POS;
                case WALL:
                    return WRIST_WALL_POS;
                case MANUAL:
                    return WRIST_REST_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum SpecimenGripperState {
        OPEN,
        CLOSED;

        public double getValue() {
            switch (this) {
                case OPEN:
                    return GRIPPER_OPEN_POS;
                case CLOSED:
                    return GRIPPER_CLOSED_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public SpecimenSubsystem(HardwareMap hardwareMap) {
        armServo = hardwareMap.get(ServoImplEx.class, HardwareMapNames.ARM_MOTOR);
        wristServo = hardwareMap.get(ServoImplEx.class, HardwareMapNames.WRIST_SERVO);
        gripperServo = hardwareMap.get(ServoImplEx.class, HardwareMapNames.GRIPPER_SERVO);

        armServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        wristServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        gripperServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        armServo.setDirection(Servo.Direction.FORWARD);
        wristServo.setDirection(Servo.Direction.FORWARD);
        gripperServo.setDirection(Servo.Direction.FORWARD);

        this.setSpecimenArmState(SpecimenArmState.MANUAL);
        this.setSpecimenGripperState(SpecimenGripperState.CLOSED);
    }

    public void setSpecimenArmState(SpecimenArmState specimenArmState) {
        this.specimenArmState = specimenArmState;
        armServo.setPosition(specimenArmState.getArmValue());
        wristServo.setPosition(specimenArmState.getWristValue());
    }

    public void setSpecimenGripperState(SpecimenGripperState specimenGripperState) {
        this.specimenGripperState = specimenGripperState;
        gripperServo.setPosition(specimenGripperState.getValue());
    }

    public SpecimenGripperState getSpecimenGripperState() {
        return specimenGripperState;
    }

    public SpecimenArmState getSpecimenArmState() {
        return specimenArmState;
    }

    public void manualAdjustArm(double delta) {
        setSpecimenArmState(SpecimenArmState.MANUAL);
        armValue = Range.clip(armValue + delta, 0.0, 1.0);
        armServo.setPosition(armValue);
    }

    public void manualAdjustWrist(double delta) {
        setSpecimenArmState(SpecimenArmState.MANUAL);
        wristValue = Range.clip(wristValue + delta, 0.0, 1.0);
        wristServo.setPosition(wristValue);
    }
}
