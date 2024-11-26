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
    public static double ARM_TRANSFERRING_POSITION = 0;
    public static double ARM_ATTACHING_POSITION = 0;
    public static double WRIST_TRANSFERRING_POSITION = 0;
    public static double WRIST_ATTACHING_POSITION = 0;
    public static double GRIPPER_OPEN = 0;
    public static double GRIPPER_CLOSED = 0;
    public static double P = 0.005;

    private SpecimenPosition specimenPosition;
    private GripperPosition gripperPosition;
    public SpecimenSubsystem(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorImpl.class, HardwareMapNames.ARM_SERVO);
        wristServo = hardwareMap.get(Servo.class, HardwareMapNames.WRIST_SERVO);
        gripperServo = hardwareMap.get(Servo.class, HardwareMapNames.GRIPPER_SERVO);
        specimenPosition = SpecimenPosition.TRANSFERRING;
        gripperPosition = GripperPosition.CLOSED;
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSpecimenPosition(SpecimenPosition position) {
        specimenPosition = position;
    }

    public void setGripper(GripperPosition position) {
        gripperPosition = position;
    }

    public void periodic() {
//        armMotor.setPosition(specimenPosition.armPosition);
        armMotor.setPower(P * (specimenPosition.armPosition - armMotor.getCurrentPosition()));
        wristServo.setPosition(specimenPosition.wristPosition);
        gripperServo.setPosition(gripperPosition.gripper);
    }

    public enum SpecimenPosition {
        TRANSFERRING(ARM_TRANSFERRING_POSITION, WRIST_TRANSFERRING_POSITION),
        ATTACHING(ARM_ATTACHING_POSITION, WRIST_ATTACHING_POSITION);
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
