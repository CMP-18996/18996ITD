package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

public class HangSubsystem extends SubsystemBase {
    public DcMotorEx hangMotor;
    private HangState hangState = HangState.OFF;

    public enum HangState {
        UP,
        DOWN,
        OFF,
        HOLD;

        public double getValue() {
            switch (this) {
                case UP:
                    return 1.0;
                case DOWN:
                    return -1.0;
                case OFF:
                    return 0;
                case HOLD:
                    return -0.5;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public void setHangState(HangState hangState) {
        this.hangState = hangState;
        hangMotor.setPower(hangState.getValue());
    }

    public HangState getHangState() {
        return hangState;
    }

    public HangSubsystem(HardwareMap hardwareMap) {
        hangMotor = hardwareMap.get(DcMotorEx.class, HardwareMapNames.HANG_MOTOR);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

