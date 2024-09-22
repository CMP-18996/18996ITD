package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    double trapdoorMinRot;
    double trapdoorMaxRot;
    double intakeRotationMinRot;
    double intakeRotationMaxRot;
    CRServo intakeServo1, intakeServo2;
    ServoEx trapdoorServo, intakeRotationServo;
    public IntakeSubsystem(HardwareMap hardwareMap) {
        trapdoorServo = new SimpleServo(hardwareMap, "trapdoor", trapdoorMinRot, trapdoorMaxRot);
        intakeRotationServo = new SimpleServo(hardwareMap, "intakeRotator", intakeRotationMinRot, intakeRotationMaxRot);
        intakeServo1 = hardwareMap.get(CRServo.class, "intake1");
        intakeServo2 = hardwareMap.get(CRServo.class, "intake2");
    }
}
