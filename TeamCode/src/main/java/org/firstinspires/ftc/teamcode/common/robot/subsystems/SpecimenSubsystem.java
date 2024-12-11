package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class SpecimenSubsystem extends SubsystemBase {
    private Servo wristServo, gripperServo;
    public DcMotorImpl armMotor;

    // (motor encoder values)
    public static int ARM_CHAMBER_POSITION = 600;
    public static int ARM_WALL_POSITION = 50;
    public static int ARM_REST_POSITION = 50;

    // (servo values)
    public static double WRIST_CHAMBER_POSITION = 0.73;
    public static double WRIST_WALL_POSITION = 0.24;
    public static double WRIST_REST_POSITION = 0.6;

    public static double GRIPPER_OPEN = 0.5;
    public static double GRIPPER_CLOSED = 0.82;
    private static int armTarget;
    private static double wristTarget;

    public static double Kp = 0.008;
    public static double Kd = -0.0007;
    public static double Ki = 0.02;

    public static double Kg = 0.2;

    public static double MAX_EXTENSION_SPEED = 1.0;
    public static double MAX_RETURN_SPEED = 1.0;

    int lastError = 0;

    public double power;
    public int error;

    double integralSum = 0;
    ElapsedTime timer = new ElapsedTime();

    public SpecimenPosition specimenPosition;
    public GripperPosition gripperPosition;

    public SpecimenSubsystem(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorImpl.class, HardwareMapNames.ARM_MOTOR);
        wristServo = hardwareMap.get(Servo.class, HardwareMapNames.WRIST_SERVO);
        gripperServo = hardwareMap.get(Servo.class, HardwareMapNames.GRIPPER_SERVO);

        specimenPosition = SpecimenPosition.CHAMBER;
        gripperPosition = GripperPosition.OPEN;

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        wristServo.setDirection(Servo.Direction.FORWARD);

        armTarget = specimenPosition.armPosition;
        wristTarget = specimenPosition.wristPosition;
    }

    public GripperPosition getGripperPosition() {
        return gripperPosition;
    }

    public void setSpecimenPosition(SpecimenPosition position) {
        this.specimenPosition = position;
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    public SpecimenPosition getSpecimenState() {
        return specimenPosition;
    }

    public void setGripperState(GripperPosition position) {
        this.gripperPosition = position;
        gripperServo.setPosition(position.val);
    }

    public void manualAdjustArm(int delta) {
        setSpecimenPosition(SpecimenPosition.MANUAL);
        armTarget += delta;
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    public void manualAdjustWrist(double delta) {
        setSpecimenPosition(SpecimenPosition.MANUAL);
        wristTarget += delta;
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    public int getError() {
        return armMotor.getCurrentPosition() - armTarget;
    }

    @Override
    public void periodic() {

//        armMotor.setPosition(specimenPosition.armPosition);
        if (specimenPosition != SpecimenPosition.MANUAL) {
            armTarget = specimenPosition.armPosition;
            wristTarget = specimenPosition.wristPosition;
        }

        double angleFromTicks = -( (360 * (armMotor.getCurrentPosition())) /1503.6) - 2;
        double G = -Kg * Math.cos(Math.toRadians(angleFromTicks));

        error = armTarget + armMotor.getCurrentPosition();
        double P = Kp * -error;

        double D = Kd * (error - lastError) / timer.seconds();

        integralSum = integralSum + (error * timer.seconds());
        double I = Ki * -integralSum;

        /*
        if (angleFromTicks > 90) {
            I = 0;
        }
        */

        lastError = error;
        timer.reset();

        power = Range.clip(P + D + I + G, -MAX_EXTENSION_SPEED, MAX_RETURN_SPEED);
//        power = 0;
        armMotor.setPower(power);
        wristServo.setPosition(wristTarget);
        /*armMotor.setPower(Arm_P * (armTarget - armMotor.getCurrentPosition()));
        wristServo.setPosition(wristTarget);
        gripperServo.setPosition(gripperPosition.gripper);*/
    }

    public enum SpecimenPosition {
        CHAMBER(ARM_CHAMBER_POSITION, WRIST_CHAMBER_POSITION),
        WALL(ARM_WALL_POSITION, WRIST_WALL_POSITION),
        REST(ARM_REST_POSITION, WRIST_REST_POSITION),
        MANUAL(0, 0.0);
        public final int armPosition;
        public final double wristPosition;
        SpecimenPosition(int arm, double wrist) {
            this.armPosition = arm;
            this.wristPosition = wrist;
        }
    }

    public enum GripperPosition {
        OPEN(GRIPPER_OPEN),
        CLOSED(GRIPPER_CLOSED);

        public double val;
        GripperPosition(double position) {
            this.val = position;
        }
    }
}
