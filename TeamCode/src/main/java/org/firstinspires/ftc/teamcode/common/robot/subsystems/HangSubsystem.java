package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

public class HangSubsystem extends SubsystemBase {
    public DcMotorEx hangMotor;
    public static int L3_POSITION = 2000; // 9517 or 1000 - thanks Arjun!
    public static int L2_POSITION = 0;
    public static int L3_HANGED_POSITION = 0;
    public static int L2_HANGED_POSITION = 0;
    public static int DOWN_POSITION = 0;
    private HangPosition target = HangPosition.DOWN;

    public HangSubsystem(HardwareMap hardwareMap) {
        hangMotor = hardwareMap.get(DcMotorEx.class, HardwareMapNames.HANG_MOTOR_1);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.updatePosition(HangPosition.DOWN);
    }

    public HangPosition getState() {
        return target;
    }

    public int getCurrentPosition() {
        return hangMotor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return target.position;
    }

    public void updatePosition(HangPosition position) {
        target = position;
    }

    public int getError() {
        return target.position - hangMotor.getCurrentPosition();
    }

    @Override
    public void periodic() {
        if (Math.abs(target.position - hangMotor.getCurrentPosition()) > 5) {
            hangMotor.setPower(-1 * Math.signum(target.position - hangMotor.getCurrentPosition()));
        }
        else {
            hangMotor.setPower(0);
        }
    }

    public enum HangPosition {
        DOWN(DOWN_POSITION),
        L2(L2_POSITION),
        L2_HUNG(L2_HANGED_POSITION),
        L3(L3_POSITION),
        L3_HUNG(L3_HANGED_POSITION);

         private int position;
         HangPosition(int position) {
             this.position = position;
         }
    }
}
