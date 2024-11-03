package org.firstinspires.ftc.teamcode.odo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

public class Drive {
    private HardwareMap hardwareMap;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    public Drive(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        leftFrontDrive = hardwareMap.get(DcMotor.class, HardwareMapNames.LEFT_FRONT);
        leftBackDrive = hardwareMap.get(DcMotor.class, HardwareMapNames.LEFT_BACK);
        rightFrontDrive = hardwareMap.get(DcMotor.class, HardwareMapNames.RIGHT_FRONT);
        rightBackDrive = hardwareMap.get(DcMotor.class, HardwareMapNames.RIGHT_BACK);

        configureMotors();
    }


    public void vectorDrive(double x, double y, double turn) {
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower  = power + cos/max + turn;
        double rightFrontPower = power - sin/max - turn;
        double leftBackPower   = power - sin/max + turn;
        double rightBackPower  = power + cos/max - turn;

        if ((power + Math.abs(turn)) > 1) {
            leftFrontPower  /= power + turn;
            rightFrontPower /= power + turn;
            leftBackPower   /= power + turn;
            rightBackPower  /= power + turn;
        }

        setMotorPowers(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    public void setMotorPowers(double lF, double lB, double rF, double rB) {
        leftFrontDrive.setPower(lF);
        rightFrontDrive.setPower(rF);
        leftBackDrive.setPower(lB);
        rightBackDrive.setPower(rB);
    }

    private void configureMotors() {
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
