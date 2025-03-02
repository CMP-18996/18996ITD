package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.common.robot.Color;
import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double ARM_PICK_UP_POS = 0.5;
    public static double ARM_MOVING_POS = 0.4;
    public static double ARM_REST_POS = 0.2;
    public static double ARM_EJECT_POS = 0.5;
    public static double ARM_FLOOR_POS = 0.6;
    public static double ARM_TRANSFER_POS = 0.1;

    public static double WRIST_PICK_UP_POS = 0.15;
    public static double WRIST_MOVING_POS = 0.1;
    public static double WRIST_REST_POS = 0.1;
    public static double WRIST_EJECT_POS = 0.6;
    public static double WRIST_FLOOR_POS = 0.8;
    public static double WRIST_TRANSFER_POS = 0.8;
    public static double WRIST_BUCKET_POS = 0.9;

    public static double PIVOT_0_POS = 0.02;
    public static double PIVOT_90_POS = 0.35;
    public static double PIVOT_TRANSFER_POS = 0.65;
    public static double PIVOT_LOCK_POS = 0.0;

    public static double ROLLER_ACTIVE = 1.0;
    public static double ROLLER_HOLD = 0.1;
    public static double ROLLER_DISABLED = 0.0;
    public static double ROLLER_REVERSING = -1.0;
    public static double ROLLER_TRANSFER = -0.1;

    public static double CLAW_OPEN_POS = 0.35;
    public static double CLAW_CLOSED_POS = 0.5;

    public static double ALPHA_CUTOFF = 200; // change to distance

    public static int colorUpdatePeriod = 1000;

    private static final Position RED = colorMidpoint(
            new Position(null, 1, 1, 1, 0),
            new Position(null, 1, 1, 1, 0),
            new Position(null, 1, 1, 1, 0));

    private static final Position YELLOW = colorMidpoint(
            new Position(null, 1, 1, 1, 0),
            new Position(null, 1, 1, 1, 0),
            new Position(null, 1, 1, 1, 0));

    private static final Position BLUE = colorMidpoint(
            new Position(null, 1, 1, 1, 0),
            new Position(null, 1, 1, 1, 0),
            new Position(null, 1, 1, 1, 0));

    private final CRServoImplEx intakeRollerServo1;
    private final CRServoImplEx intakeRollerServo2;
    private final ServoImplEx intakePivotServo;
    private final ServoImplEx intakeArmServo;
    private final ServoImplEx intakeWristServo;
    private final ServoImplEx intakeClawServo;

    private final RevColorSensorV3 colorSensor;

    private IntakeRollerState intakeRollerState;
    private IntakePivotState intakePivotState;
    private IntakeArmState intakeArmState;
    private IntakeWristState intakeWristState;
    private IntakeClawState intakeClawState;

    private Color currentColor = Color.NONE;
    private Color previousColor = Color.NONE;
    private ColorSensorStatus colorSensorStatus;

    private ElapsedTime elapsedTime = new ElapsedTime();

    public enum IntakeRollerState {
        ACTIVE,
        HOLD,
        DISABLED,
        REVERSING,
        TRANSFER;

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
                case TRANSFER:
                    return ROLLER_TRANSFER;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum IntakeArmState {
        REST,
        MOVING,
        PICK_UP,
        EJECT,
        FLOOR,
        TRANSFER;

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
                case FLOOR:
                    return ARM_FLOOR_POS;
                case TRANSFER:
                    return ARM_TRANSFER_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum IntakeWristState {
        REST,
        MOVING,
        PICK_UP,
        EJECT,
        FLOOR,
        TRANSFER,
        BUCKET;

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
                case FLOOR:
                    return WRIST_FLOOR_POS;
                case TRANSFER:
                    return WRIST_TRANSFER_POS;
                case BUCKET:
                    return WRIST_BUCKET_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum IntakePivotState {
        PIVOT_0,
        PIVOT_90,
        PIVOT_LOCK,
        PIVOT_TRANSFER;

        public double getValue() {
            switch (this) {
                case PIVOT_0:
                    return PIVOT_0_POS;
                case PIVOT_90:
                    return PIVOT_90_POS;
                case PIVOT_LOCK:
                    return PIVOT_LOCK_POS;
                case PIVOT_TRANSFER:
                    return PIVOT_TRANSFER_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum IntakeClawState {
        OPEN,
        CLOSED;

        public double getValue() {
            switch (this) {
                case OPEN:
                    return CLAW_OPEN_POS;
                case CLOSED:
                    return CLAW_CLOSED_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum ColorSensorStatus {
        ENABLED,
        DISABLED
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeRollerServo1 = hardwareMap.get(CRServoImplEx.class, HardwareMapNames.INTAKE_ROLLER_1);
        intakeRollerServo2 = hardwareMap.get(CRServoImplEx.class, HardwareMapNames.INTAKE_ROLLER_2);

        intakePivotServo = hardwareMap.get(ServoImplEx.class, HardwareMapNames.INTAKE_PIVOT);
        intakeArmServo = hardwareMap.get(ServoImplEx.class, HardwareMapNames.INTAKE_BOTTOM_PIVOT);
        intakeWristServo = hardwareMap.get(ServoImplEx.class, HardwareMapNames.INTAKE_TOP_PIVOT);
        intakeClawServo = hardwareMap.get(ServoImplEx.class, HardwareMapNames.INTAKE_CLAW);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, HardwareMapNames.INTAKE_COLOR_SENSOR);

        intakeRollerServo1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeRollerServo2.setPwmRange(new PwmControl.PwmRange(500, 2500));

        intakePivotServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeArmServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeWristServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeClawServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        intakeRollerServo1.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRollerServo2.setDirection(DcMotorSimple.Direction.REVERSE);

        intakePivotServo.setDirection(Servo.Direction.FORWARD);
        intakeArmServo.setDirection(Servo.Direction.FORWARD);
        intakeWristServo.setDirection(Servo.Direction.FORWARD);
        intakeClawServo.setDirection(Servo.Direction.FORWARD);

        colorSensor.enableLed(true);

        this.setIntakeRollerState(IntakeRollerState.DISABLED);
        this.setIntakePivotState(IntakePivotState.PIVOT_0);
        this.setIntakeArmState(IntakeArmState.REST);
        this.setIntakeWristState(IntakeWristState.REST);
        this.setIntakeClawState(IntakeClawState.CLOSED);
        this.setColorSensorStatus(ColorSensorStatus.ENABLED);
    }

    public void setColorSensorStatus(ColorSensorStatus colorSensorStatus) {
        this.colorSensorStatus = colorSensorStatus;
    }

    public void setIntakeRollerState(IntakeRollerState intakeRollerState) {
        this.intakeRollerState = intakeRollerState;
        intakeRollerServo1.setPower(intakeRollerState.getValue());
        intakeRollerServo2.setPower(intakeRollerState.getValue());
    }

    public void setIntakePivotState(IntakePivotState intakePivotState) {
        this.intakePivotState = intakePivotState;
        intakePivotServo.setPosition(intakePivotState.getValue());
    }

    public void setIntakeLockAngle(double degrees) {
        PIVOT_LOCK_POS = (degrees + 90) / 180;
        intakePivotState = IntakePivotState.PIVOT_LOCK;
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

    public void setIntakeClawState(IntakeClawState intakeClawState) {
        this.intakeClawState = intakeClawState;
        intakeClawServo.setPosition(intakeClawState.getValue());
    }

    public ColorSensorStatus getColorSensorStatus() {
        return colorSensorStatus;
    }

    public IntakePivotState getIntakePivotState() { return intakePivotState; }

    public IntakeArmState getIntakeArmState() { return intakeArmState; }

    public IntakeWristState getIntakeWristState() {
        return intakeWristState;
    }

    public IntakeClawState getIntakeClawState() {
        return intakeClawState;
    }

    public IntakeRollerState getIntakeRollerState() {
        return intakeRollerState;
    }

    public Color getCurrentColor() {
        return currentColor;
    }

    public Color getPreviousColor() { return previousColor;}

    private void updateCurrentColor() {
        previousColor = currentColor;

        if(colorSensorStatus.equals(ColorSensorStatus.ENABLED) && colorSensor.alpha() > ALPHA_CUTOFF) {
            Position color = new Position(DistanceUnit.INCH, colorSensor.red(), colorSensor.green(), colorSensor.blue(), 0);
            double r = colorDistance(color, RED);
            double y = colorDistance(color, YELLOW);
            double b = colorDistance(color, BLUE);

            if(r < y & r < b) {
                currentColor = Color.RED;
            }
            else if(y < r & y < b) {
                currentColor = Color.YELLOW;
            }
            else if(b < y & b < r) {
                currentColor = Color.BLUE;
            }
        }
        else {
            currentColor = Color.NONE;
        }
    }

    @Override
    public void periodic() {
        if(elapsedTime.milliseconds() > colorUpdatePeriod) {
            elapsedTime.reset();
            updateCurrentColor();
        }
    }

    // move these to another class
    public static double colorDistance(Position p1, Position p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dz = p2.z - p1.z;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    public static Position colorMidpoint(Position... points) {
        double sumX = 0, sumY = 0, sumZ = 0;
        int count = points.length;

        for (Position p : points) {
            sumX += p.x;
            sumY += p.y;
            sumZ += p.z;
        }

        return new Position(DistanceUnit.INCH, sumX / count, sumY / count, sumZ / count, 0);
    }
}
