package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    // Constants
    final static double TRAPDOOR_MIN_ROT = 0.0;
    final static double TRAPDOOR_MAX_ROT = 0.0;
    final static double INTAKE_ROTATION_MIN_ROT = 0.0; // max and min rotation used as what arm is actually being rotated to, subject to change
    final static double INTAKE_ROTATION_MAX_ROT = 0.0;
    public static double CLOSED_VALUE = .5;
    public static double EJECTING_VALUE = 1.0;
    public static double TRANSFERRING_VALUE = 0;
    public static double ACTIVE_VALUE = 1.0;
    public static double DISABLED_VALUE = 0.0;

    // State
    private CRServo intakeServo1, intakeServo2;
    private ServoEx trapdoorServo, intakeRotationServo;
    private TrapdoorState trapdoorState = TrapdoorState.CLOSED;
    private IntakingState intakingState = IntakingState.DISABLED;
    private IntakeRotatorState intakeRotatorState = IntakeRotatorState.TRANSFERING;
    public enum TrapdoorState {
        CLOSED(CLOSED_VALUE),
        EJECTING(EJECTING_VALUE),
        TRANSFERRING(TRANSFERRING_VALUE);
        public double trapdoorValue;
        TrapdoorState(double val) {trapdoorValue = val;}

    }
    public enum IntakingState {
        ACTIVE(ACTIVE_VALUE),
        DISABLED(DISABLED_VALUE);
        public double intakingValue;
        IntakingState(double val) {intakingValue = val;}
    }

    public enum IntakeRotatorState {
        TRANSFERING(INTAKE_ROTATION_MIN_ROT),
        DROPPING(INTAKE_ROTATION_MAX_ROT);
        public double rotationValue;
        IntakeRotatorState(double val) {rotationValue = val;}
    }

    public void updateTrapdoorState(TrapdoorState setState) {
        trapdoorState = setState;
        trapdoorServo.setPosition(trapdoorState.trapdoorValue);
    }

    public void updateIntakingState(IntakingState setState) {
        intakingState = setState;
        intakeServo1.setPower(intakingState.intakingValue);
        intakeServo2.setPower(intakingState.intakingValue);
    }

    public void updateIntakeRotatorState(IntakeRotatorState setState) {
        intakeRotatorState = setState;
        intakeRotationServo.setPosition(intakeRotatorState.rotationValue);
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {
        trapdoorServo = new SimpleServo(hardwareMap, "trapdoor", TRAPDOOR_MIN_ROT, TRAPDOOR_MAX_ROT);
        intakeRotationServo = new SimpleServo(hardwareMap, "intakeRotator", INTAKE_ROTATION_MIN_ROT, INTAKE_ROTATION_MAX_ROT);
        intakeServo1 = hardwareMap.get(CRServo.class, "intake1");
        intakeServo2 = hardwareMap.get(CRServo.class, "intake2");
        intakeServo1.setDirection(DcMotorSimple.Direction.FORWARD); // subject to change
        intakeServo1.setDirection(DcMotorSimple.Direction.FORWARD); // subject to change
    }
}
