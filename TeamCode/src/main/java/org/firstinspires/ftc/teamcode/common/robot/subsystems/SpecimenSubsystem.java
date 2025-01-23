package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class SpecimenSubsystem extends SubsystemBase {
    public static double Kp = 0.008;
    public static double Kd = 0.0012;
    public static double Ki = 0.02;
    public static double Kg = -0.35;
    public static int INTEGRAL_ENABLE_POINT = 10;

    public double angleFromTicks;

    public static int ARM_CHAMBER_POS = 270;
    public static int ARM_WALL_POS = 20;

    public static double WRIST_CHAMBER_POS = 0.8;
    public static double WRIST_WALL_POS = 0.1;

    public static double GRIPPER_OPEN_POS = 0.5;
    public static double GRIPPER_CLOSED_POS = 0.82;

    public static double MAX_EXTENSION_SPEED = 0.8;
    public static double MAX_RETURN_SPEED = 0.7;

    // Used because of manual adjustment
    private int armTarget;
    private double wristTarget;

    private final Servo wristServo;
    private final Servo gripperServo;
    private final DcMotorEx armMotor;

    private SpecimenArmState specimenArmState;
    private SpecimenGripperState specimenGripperState;

    private int lastError = 0;
    private double integralSum = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public enum SpecimenArmState {
        CHAMBER,
        WALL,
        MANUAL;

        public int getArmValue() {
            switch (this) {
                case CHAMBER:
                    return ARM_CHAMBER_POS;
                case WALL:
                    return ARM_WALL_POS;
                case MANUAL:
                    return 0;
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
                    return 0;
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
        armMotor = hardwareMap.get(DcMotorEx.class, HardwareMapNames.ARM_MOTOR);
        wristServo = hardwareMap.get(Servo.class, HardwareMapNames.WRIST_SERVO);
        gripperServo = hardwareMap.get(Servo.class, HardwareMapNames.GRIPPER_SERVO);

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        wristServo.setDirection(Servo.Direction.FORWARD);
        gripperServo.setDirection(Servo.Direction.FORWARD);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.setSpecimenArmState(SpecimenArmState.MANUAL);
        this.setSpecimenGripperState(SpecimenGripperState.CLOSED);
    }

    public void setSpecimenArmState(SpecimenArmState specimenArmState) {
        this.specimenArmState = specimenArmState;
        if(!specimenArmState.equals(SpecimenArmState.MANUAL)) {
            this.armTarget = specimenArmState.getArmValue();
            integralSum = 0;
            lastError = 0;
            timer.reset();

            this.wristTarget = specimenArmState.getWristValue();
            wristServo.setPosition(specimenArmState.getWristValue());
        }
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

    public int getError() {
        return -armMotor.getCurrentPosition() - armTarget;
    }

    public void manualAdjustArm(int delta) {
        setSpecimenArmState(SpecimenArmState.MANUAL);
        armTarget += delta;
        integralSum = 0;
        //lastError = 0;
        timer.reset();
    }

    public void manualAdjustWrist(double delta) {
        setSpecimenArmState(SpecimenArmState.MANUAL);
        wristTarget += delta;
        wristServo.setPosition(wristTarget);
        integralSum = 0;
        //lastError = 0;
        timer.reset();
    }

    @Override
    public void periodic() {
        int error = getError();

        double P = Kp * error;

        if (Math.abs(error) > INTEGRAL_ENABLE_POINT) {
            integralSum = 0;
        }
        else {
            integralSum = integralSum + (error * timer.seconds());
        }

        double I = Ki * integralSum;

        double D = Kd * (error - lastError) / timer.seconds();

        angleFromTicks = -360 * armMotor.getCurrentPosition() / 751.8;
        double G = Kg * Math.cos(Math.toRadians(angleFromTicks));

        lastError = error;
        timer.reset();

        double power = Range.clip(P + D + I + G, -MAX_EXTENSION_SPEED, MAX_RETURN_SPEED);

        armMotor.setPower(power);
    }
}
