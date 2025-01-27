package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

public class HangSubsystem extends SubsystemBase {
    public DcMotorEx hangMotor;
    public static double CUSTOM_HANG_VALUE = .6;
    private HangState state = HangState.OFF;


    public HangState getState() {
        return state;
    }
    public void setState(HangState newState) {
        state = newState;
    }


    @Override
    public void periodic() {
        if (state != HangState.OFF) {
            hangMotor.setPower(state.power);
        }
    }

    public enum HangState {
        UP(1),
        DOWN(-1),
        OFF(0),
        CUSTOM(CUSTOM_HANG_VALUE);

         private double power;
         HangState(double position) {
             this.power = position;
         }
    }

    public HangSubsystem(HardwareMap hardwareMap) {
        hangMotor = hardwareMap.get(DcMotorEx.class, HardwareMapNames.HANG_MOTOR_1);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

