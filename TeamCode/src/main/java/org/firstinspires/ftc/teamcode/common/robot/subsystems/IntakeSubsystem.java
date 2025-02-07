package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double ARM_TRANSFER_POS = 0.0; // max and min rotation used as what arm is actually being rotated to, subject to change
    public static double ARM_PICK_UP_POS = 1.0;
    public static double ARM_MOVING_POS = 0.7;
    public static double ARM_REST_POS = 0.15;
    public static double ARM_REJECT_POS = 0.6;

    public static double WRIST_TRANSFER_POS = 0.2;
    public static double WRIST_PICK_UP_POS = 1.0;
    public static double WRIST_MOVING_POS = 0.65;
    public static double WRIST_REST_POS = 0.2;
    public static double WRIST_REJECT_POS = 0.6;

    public static double TRAPDOOR_CLOSED_POS = 0.3;
    public static double TRAPDOOR_OPEN_POS = 0.8;

    public static double ROLLER_ACTIVE = 1.0;
    public static double ROLLER_DISABLED = 0.0;
    public static double ROLLER_HOLD = 0.4;
    public static double ROLLER_REVERSING = -1.0;

    public static double ALPHA_CUTOFF = 300;

    private final DcMotorEx intakeMotor;
    private final Servo trapdoorServo;
    private final Servo intakeArmServo;
    private final Servo intakeWristServo;
    private final ColorSensor colorSensor;

    private IntakeTrapdoorState intakeTrapdoorState;
    private IntakeMotorState intakeMotorState;
    private IntakeArmState intakeArmState;
    private IntakeWristState intakeWristState;

    private Color currentColor;
    private ColorSensorStatus colorSensorStatus;

    public enum IntakeMotorState {
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
        TRANSFER,
        REST,
        MOVING,
        PICK_UP,
        REJECT;

        public double getValue() {
            switch (this) {
                case TRANSFER:
                    return ARM_TRANSFER_POS;
                case REST:
                    return ARM_REST_POS;
                case MOVING:
                    return ARM_MOVING_POS;
                case PICK_UP:
                    return ARM_PICK_UP_POS;
                case REJECT:
                    return ARM_REJECT_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum IntakeWristState {
        TRANSFER,
        REST,
        MOVING,
        PICK_UP,
        REJECT;

        public double getValue() {
            switch (this) {
                case TRANSFER:
                    return WRIST_TRANSFER_POS;
                case REST:
                    return WRIST_REST_POS;
                case MOVING:
                    return WRIST_MOVING_POS;
                case PICK_UP:
                    return WRIST_PICK_UP_POS;
                case REJECT:
                    return WRIST_REJECT_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum IntakeTrapdoorState {
        CLOSED,
        OPEN;

        public double getValue() {
            switch (this) {
                case CLOSED:
                    return TRAPDOOR_CLOSED_POS;
                case OPEN:
                    return TRAPDOOR_OPEN_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum Color {
        NONE,
        YELLOW,
        RED,
        BLUE
    }

    public enum ColorSensorStatus {
        ENABLED,
        DISABLED
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {
        trapdoorServo = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_TRAPDOOR);
        intakeArmServo = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_BOTTOM_PIVOT);
        intakeWristServo = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_TOP_PIVOT);
        intakeMotor = hardwareMap.get(DcMotorEx.class, HardwareMapNames.INTAKE_MOTOR);
        colorSensor = hardwareMap.get(ColorSensor.class, HardwareMapNames.INTAKE_COLOR_SENSOR);

        trapdoorServo.setDirection(Servo.Direction.FORWARD);
        intakeArmServo.setDirection(Servo.Direction.FORWARD);
        intakeWristServo.setDirection(Servo.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        colorSensor.enableLed(true);

        intakeWristServo.scaleRange(0.0, 0.8);

        this.setTrapdoorState(IntakeTrapdoorState.CLOSED);
        this.setIntakeArmState(IntakeArmState.REST);
        this.setIntakeWristState(IntakeWristState.REST);
        this.setIntakeMotorState(IntakeMotorState.DISABLED);
        this.setColorSensorStatus(ColorSensorStatus.ENABLED);
    }

    public void setColorSensorStatus(ColorSensorStatus colorSensorStatus) {
        this.colorSensorStatus = colorSensorStatus;
    }

    public void setTrapdoorState(IntakeTrapdoorState intakeTrapdoorState) {
        this.intakeTrapdoorState = intakeTrapdoorState;
        trapdoorServo.setPosition(intakeTrapdoorState.getValue());
    }

    public void setIntakeArmState(IntakeArmState intakeArmState) {
        this.intakeArmState = intakeArmState;
        intakeArmServo.setPosition(intakeArmState.getValue());
    }

    public void setIntakeWristState(IntakeWristState intakeWristState) {
        this.intakeWristState = intakeWristState;
        intakeWristServo.setPosition(intakeWristState.getValue());
    }

    public void adjustWristPosition(double delta) {
        intakeWristServo.setPosition(intakeWristState.getValue() + delta);
    }

    public void setIntakeMotorState(IntakeMotorState intakeMotorState) {
        this.intakeMotorState = intakeMotorState;
        intakeMotor.setPower(intakeMotorState.getValue());
    }

    public ColorSensorStatus getColorSensorStatus() {
        return colorSensorStatus;
    }

    public IntakeTrapdoorState getTrapdoorState() {
        return intakeTrapdoorState;
    }

    public IntakeArmState getIntakeArmState() { return intakeArmState; }

    public IntakeWristState getIntakeWristState() {
        return intakeWristState;
    }

    public IntakeMotorState getIntakeRollerState() {
        return intakeMotorState;
    }

    public Color getCurrentColor() {
        return currentColor;
    }

    private void updateCurrentColor() {
        if(colorSensorStatus.equals(IntakeSubsystem.ColorSensorStatus.DISABLED)) {
            currentColor = Color.NONE;
        }
        else if(colorSensor.alpha() < ALPHA_CUTOFF) {
            currentColor = Color.NONE;
        }
        // DO NOT CHANGE THE ORDER OF THESE IF STATEMENTS, COLOR DETECTION WILL BREAK
        else if(colorSensor.blue() > 500) {
            currentColor = Color.BLUE;
        }
        else if(colorSensor.green() > 600) {
            currentColor = Color.YELLOW;
        }
        else if(colorSensor.red() > 600) {
            currentColor = Color.RED;
        }
        else {
            currentColor = IntakeSubsystem.Color.NONE;
        }
    }

    @Override
    public void periodic() {
        updateCurrentColor();
    }
}
