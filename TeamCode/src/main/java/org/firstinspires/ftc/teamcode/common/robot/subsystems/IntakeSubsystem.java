package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double ARM_TRANSFER_POS = 0.0; // max and min rotation used as what arm is actually being rotated to, subject to change
    public static double ARM_PICK_UP_POS = 0.82;
    public static double ARM_MOVING_POS = 0.58;
    public static double ARM_REST_POS = 0.15;

    public static double WRIST_TRANSFER_POS = 0.875/5;
    public static double WRIST_PICK_UP_POS = 0.6/5;
    public static double WRIST_MOVING_POS = 0.8/5;
    public static double WRIST_REST_POS = 1.0/5;

    public static double TRAPDOOR_CLOSED_POS = 0.3;
    public static double TRAPDOOR_OPEN_POS = 0.8;

    public static double ROLLER_ACTIVE = 1.0;
    public static double ROLLER_DISABLED = 0.0;
    public static double ROLLER_REVERSING = -1.0;

    public static double ALPHA_CUTOFF = 280;

    private final CRServo intakeRollerServo;
    private final Servo trapdoorServo;
    private final Servo intakeArmServo;
    private final Servo intakeWristServo;
    private final ColorSensor colorSensor;

    private TrapdoorState trapdoorState;
    private IntakeRollerState intakeRollerState;
    private IntakeArmState intakeArmState;
    private IntakeWristState intakeWristState;

    private Color currentColor;
    private ColorSensorStatus colorSensorStatus;

    public enum IntakeRollerState {
        ACTIVE,
        DISABLED,
        REVERSING;

        public double getValue() {
            switch (this) {
                case ACTIVE:
                    return ROLLER_ACTIVE;
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
        PICK_UP;

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
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum IntakeWristState {
        TRANSFER,
        REST,
        MOVING,
        PICK_UP;

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
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum TrapdoorState {
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
        intakeRollerServo = hardwareMap.get(CRServo.class, HardwareMapNames.INTAKE_ROLLER_SERVO);
        colorSensor = hardwareMap.get(ColorSensor.class, HardwareMapNames.INTAKE_COLOR_SENSOR);

        trapdoorServo.setDirection(Servo.Direction.FORWARD);
        intakeArmServo.setDirection(Servo.Direction.FORWARD);
        intakeWristServo.setDirection(Servo.Direction.FORWARD);
        intakeRollerServo.setDirection(DcMotor.Direction.REVERSE);
        colorSensor.enableLed(true);

        this.setTrapdoorState(TrapdoorState.CLOSED);
        this.setIntakeArmState(IntakeArmState.REST);
        this.setIntakeWristState(IntakeWristState.REST);
        this.setIntakeRollerState(IntakeRollerState.DISABLED);
        this.setColorSensorStatus(ColorSensorStatus.ENABLED);
    }

    public void setColorSensorStatus(ColorSensorStatus colorSensorStatus) {
        this.colorSensorStatus = colorSensorStatus;
    }

    public void setTrapdoorState(TrapdoorState trapdoorState) {
        this.trapdoorState = trapdoorState;
        trapdoorServo.setPosition(trapdoorState.getValue());
    }

    public void setIntakeArmState(IntakeArmState intakeArmState) {
        this.intakeArmState = intakeArmState;
        intakeArmServo.setPosition(intakeArmState.getValue());
    }

    public void setIntakeWristState(IntakeWristState intakeWristState) {
        this.intakeWristState = intakeWristState;
        intakeWristServo.setPosition(intakeWristState.getValue());
    }

    public void setIntakeRollerState(IntakeRollerState intakeRollerState) {
        this.intakeRollerState = intakeRollerState;
        intakeRollerServo.setPower(intakeRollerState.getValue());
    }

    public ColorSensorStatus getColorSensorStatus() {
        return colorSensorStatus;
    }

    public TrapdoorState getTrapdoorState() {
        return trapdoorState;
    }

    public IntakeArmState getIntakeArmState() { return intakeArmState; }

    public IntakeWristState getIntakeWristState() {
        return intakeWristState;
    }

    public IntakeRollerState getIntakeRollerState() {
        return intakeRollerState;
    }

    public Color getCurrentColor() {
        return currentColor;
    }

    private void updateCurrentColor() {
        /*
        int color = colorSensor.argb();
        int r, g, b, a;
        a = (color >> 24) & 0xFF;
        r = (color >> 16) & 0xFF;
        g = (color >> 8) & 0xFF;
           b?

         */
        int r, g, b, a;
        a = colorSensor.alpha();
        r = colorSensor.red();
        g = colorSensor.green();
        b = colorSensor.blue();

        if(colorSensorStatus.equals(IntakeSubsystem.ColorSensorStatus.DISABLED) || a < ALPHA_CUTOFF) {
            currentColor = IntakeSubsystem.Color.NONE;
        }
        else if(r > g && r > b){
            currentColor = IntakeSubsystem.Color.RED;
        }
        else if(g > r && g > b){
            currentColor = IntakeSubsystem.Color.YELLOW;
        }
        else if(b > r && b > g){
            currentColor = IntakeSubsystem.Color.BLUE;
        }
        else {
            // Pray this never happens
            currentColor = IntakeSubsystem.Color.NONE;
        }
    }

    @Override
    public void periodic() {
        updateCurrentColor();
    }
}
