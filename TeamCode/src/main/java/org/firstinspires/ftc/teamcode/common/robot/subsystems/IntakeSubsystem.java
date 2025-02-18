package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double ARM_PICK_UP_POS = 0.75;
    public static double ARM_MOVING_POS = 0.6;
    public static double ARM_REST_POS = 0.2;
    public static double ARM_EJECT_POS = 0.65;

    public static double WRIST_PICK_UP_POS = 0.55;
    public static double WRIST_MOVING_POS = 0.37;
    public static double WRIST_REST_POS = 0.4;
    public static double WRIST_EJECT_POS = 0.9;

    public static double PIVOT_0_POS = 0;
    public static double PIVOT_45_POS = 0.28;
    public static double PIVOT_90_POS = 0.56;

    public static double ROLLER_ACTIVE = 1.0;
    public static double ROLLER_HOLD = 0.02;
    public static double ROLLER_DISABLED = 0.0;
    public static double ROLLER_REVERSING = -1.0;

    private final CRServo intakeRollerServo;
    private final Servo intakePivotServo;
    private final Servo intakeArmServo;
    private final Servo intakeWristServo;

    private IntakeRollerState intakeRollerState;
    private IntakePivotState intakePivotState;
    private IntakeArmState intakeArmState;
    private IntakeWristState intakeWristState;

    public enum IntakeRollerState {
        ACTIVE,
        HOLD,
        DISABLED,
        REVERSING;

        public double getValue() {
            switch (this) {
                case ACTIVE:
                    return ROLLER_ACTIVE;
                case HOLD:
                    return ROLLER_HOLD;
                case DISABLED:
                    return ROLLER_DISABLED;
                case REVERSING:
                    return ROLLER_REVERSING;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum IntakeArmState {
        REST,
        MOVING,
        PICK_UP,
        EJECT;

        public double getValue() {
            switch (this) {
                case REST:
                    return ARM_REST_POS;
                case MOVING:
                    return ARM_MOVING_POS;
                case PICK_UP:
                    return ARM_PICK_UP_POS;
                case EJECT:
                    return ARM_EJECT_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum IntakeWristState {
        REST,
        MOVING,
        PICK_UP,
        EJECT;

        public double getValue() {
            switch (this) {
                case REST:
                    return WRIST_REST_POS;
                case MOVING:
                    return WRIST_MOVING_POS;
                case PICK_UP:
                    return WRIST_PICK_UP_POS;
                case EJECT:
                    return WRIST_EJECT_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum IntakePivotState {
        PIVOT_0,
        PIVOT_45,
        PIVOT_90;

        public double getValue() {
            switch (this) {
                case PIVOT_0:
                    return PIVOT_0_POS;
                case PIVOT_45:
                    return PIVOT_45_POS;
                case PIVOT_90:
                    return PIVOT_90_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeRollerServo = hardwareMap.get(CRServo.class, HardwareMapNames.INTAKE_ROLLER);
        intakePivotServo = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_PIVOT);
        intakeArmServo = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_BOTTOM_PIVOT);
        intakeWristServo = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_TOP_PIVOT);

        intakeRollerServo.setDirection(DcMotorSimple.Direction.FORWARD);
        intakePivotServo.setDirection(Servo.Direction.FORWARD);
        intakeArmServo.setDirection(Servo.Direction.FORWARD);
        intakeWristServo.setDirection(Servo.Direction.FORWARD);

        this.setIntakeRollerState(IntakeRollerState.DISABLED);
        this.setIntakePivotState(IntakePivotState.PIVOT_0);
        this.setIntakeArmState(IntakeArmState.REST);
        this.setIntakeWristState(IntakeWristState.REST);
    }

    public void setIntakeRollerState(IntakeRollerState intakeRollerState) {
        this.intakeRollerState = intakeRollerState;
        intakeRollerServo.setPower(intakeRollerState.getValue());
    }

    public void setIntakePivotState(IntakePivotState intakePivotState) {
        this.intakePivotState = intakePivotState;
        intakePivotServo.setPosition(intakePivotState.getValue());
    }

    public void setIntakeArmState(IntakeArmState intakeArmState) {
        this.intakeArmState = intakeArmState;
        intakeArmServo.setPosition(intakeArmState.getValue());
    }

    public void setIntakeWristState(IntakeWristState intakeWristState) {
        this.intakeWristState = intakeWristState;
        intakeWristServo.setPosition(intakeWristState.getValue());
    }

    public IntakePivotState getIntakePivotState() { return intakePivotState; }

    public IntakeArmState getIntakeArmState() { return intakeArmState; }

    public IntakeWristState getIntakeWristState() {
        return intakeWristState;
    }

    public IntakeRollerState getIntakeRollerState() {
        return intakeRollerState;
    }
}
