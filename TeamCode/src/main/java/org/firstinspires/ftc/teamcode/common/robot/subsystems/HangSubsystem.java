package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HangSubsystem extends SubsystemBase {
    private DcMotorEx hangMotor;
    public HangSubsystem(HardwareMap hardwareMap) {
        hangMotor = hardwareMap.get(DcMotorEx.class, "hang");
    }
}
