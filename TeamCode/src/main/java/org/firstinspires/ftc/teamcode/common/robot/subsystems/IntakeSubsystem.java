package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    final double TRAPDOOR_MIN_ROT = 0.0;
    final double TRAPDOOR_MAX_ROT = 0.0;
    final double INTAKE_ROTATION_MIN_ROT = 0.0;
    final double INTAKE_ROTATION_MAX_ROT = 0.0;
    CRServo intakeServo1, intakeServo2;
    ServoEx trapdoorServo, intakeRotationServo;
    public static double CLOSED_VALUE = 0.1;
    public static double EJECTING_VALUE = 10.0;
    public static double TRANSFERRING_VALUE = 1.0;
    public static double ACTIVE_VALUE = 1.0;
    public static double DISABLED_VALUE = 0.0;
    private TrapdoorState trapdoorState = TrapdoorState.CLOSED;
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

    public IntakeSubsystem(HardwareMap hardwareMap) {
        trapdoorServo = new SimpleServo(hardwareMap, "trapdoor", TRAPDOOR_MIN_ROT, TRAPDOOR_MAX_ROT);
        intakeRotationServo = new SimpleServo(hardwareMap, "intakeRotator", INTAKE_ROTATION_MIN_ROT, INTAKE_ROTATION_MAX_ROT);
        intakeServo1 = hardwareMap.get(CRServo.class, "intake1");
        intakeServo2 = hardwareMap.get(CRServo.class, "intake2");
    }
    public void updateTrapdoorState(TrapdoorState setState) {
        trapdoorState = setState;
        trapdoorServo.setPosition(trapdoorState.trapdoorValue);
    }
}
