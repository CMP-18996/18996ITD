package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
public class SpecimenSubsystem extends SubsystemBase {
    private Servo wristServo, gripperServo;
    private DcMotorImpl armMotor;

    // (motor encoder values)
    public static int ARM_TRANSFERRING_POSITION = 35;
    public static int ARM_ATTACHING_POSITION = 270;

    // (servo values)
    public static double WRIST_TRANSFERRING_POSITION = 0.73;
    public static double WRIST_ATTACHING_POSITION = 0.24;

    public static double GRIPPER_OPEN = 0.5;
    public static double GRIPPER_CLOSED = 0.82;

    public static double Arm_P = 0.005;

    private static double armTarget;
    private static double wristTarget;

    private SpecimenPosition specimenPosition;
    private GripperPosition gripperPosition;

    public SpecimenSubsystem(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorImpl.class, HardwareMapNames.ARM_MOTOR);
        wristServo = hardwareMap.get(Servo.class, HardwareMapNames.WRIST_SERVO);
        gripperServo = hardwareMap.get(Servo.class, HardwareMapNames.GRIPPER_SERVO);

        specimenPosition = SpecimenPosition.TRANSFERRING;
        gripperPosition = GripperPosition.CLOSED;

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armTarget = specimenPosition.armPosition;
        wristTarget = specimenPosition.wristPosition;
    }

    public void setSpecimenPosition(SpecimenPosition position) {
        specimenPosition = position;
    }

    public void setGripper(GripperPosition position) {
        gripperPosition = position;
    }

    public void manualAdjustArm(int delta) {
        setSpecimenPosition(SpecimenPosition.MANUAL);
        armTarget += delta;
    }

    public void manualAdjustWrist(double delta) {
        setSpecimenPosition(SpecimenPosition.MANUAL);
        wristTarget += delta;
    }

    public void periodic() {
//        armMotor.setPosition(specimenPosition.armPosition);
        if (specimenPosition != SpecimenPosition.MANUAL) {
            armTarget = specimenPosition.armPosition;
            wristTarget = specimenPosition.wristPosition;
        }

        armMotor.setPower(Arm_P * (armTarget - armMotor.getCurrentPosition()));
        wristServo.setPosition(wristTarget);
        gripperServo.setPosition(gripperPosition.gripper);
    }

    public enum SpecimenPosition {
        TRANSFERRING(ARM_TRANSFERRING_POSITION, WRIST_TRANSFERRING_POSITION),
        ATTACHING(ARM_ATTACHING_POSITION, WRIST_ATTACHING_POSITION),
        MANUAL(0, 0);
        public double armPosition, wristPosition;
        SpecimenPosition(double arm, double wrist) {
            this.armPosition = arm;
            this.wristPosition = wrist;
        }
    }

    public enum GripperPosition {
        OPEN(GRIPPER_OPEN),
        CLOSED(GRIPPER_CLOSED);

        public double gripper;
        GripperPosition(double position) {
            this.gripper = position;
        }
    }
}
