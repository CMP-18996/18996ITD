package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HangSubsystem extends SubsystemBase {
    private DcMotorEx hangMotor;
    public static int UP_POSITION = 3600;
    public static int DOWN_POSITION = 0;
    private int target = 0;

    public HangSubsystem(HardwareMap hardwareMap) {
        //hangMotor = hardwareMap.get(DcMotorEx.class, HardwareMapNames.HANG_MOTOR_1);
        //hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    /*
    public void updatePosition(int position) {
        target = position;
    }

    @Override
    public void periodic() {
        if (target - hangMotor.getCurrentPosition() > 10) {
            hangMotor.setPower(1);
        }
    } */
}
