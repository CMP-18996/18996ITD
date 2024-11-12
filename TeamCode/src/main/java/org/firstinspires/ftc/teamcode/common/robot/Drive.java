package org.firstinspires.ftc.teamcode.common.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.jetbrains.annotations.NotNull;

public class Drive {
    private HardwareMap hardwareMap;

    private final DcMotor leftFrontDrive;
    private final DcMotor leftBackDrive;
    private final DcMotor rightFrontDrive;
    private final DcMotor rightBackDrive;

    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC;
    }

    private DriveMode driveMode = DriveMode.ROBOT_CENTRIC;

    public Drive(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        leftFrontDrive = hardwareMap.get(DcMotor.class, HardwareMapNames.LEFT_FRONT);
        leftBackDrive = hardwareMap.get(DcMotor.class, HardwareMapNames.LEFT_BACK);
        rightFrontDrive = hardwareMap.get(DcMotor.class, HardwareMapNames.RIGHT_FRONT);
        rightBackDrive = hardwareMap.get(DcMotor.class, HardwareMapNames.RIGHT_BACK);

        configureMotors();
    }

    /*
    *  X and Y are the components of the linear movement vector [-1.0, 1.0]
    *  Turn is relative speed to rotate the robot [-1.0, 1.0]
    *  CurrentHeading is the robot's current heading in RADIANS [-pi, pi]
     */
    public void vectorDrive(double x, double y, double turn, double currentHeading){
        if (driveMode == DriveMode.ROBOT_CENTRIC) {
            calculateMotorPowers(x, y, turn);
        }
        else if (driveMode == DriveMode.FIELD_CENTRIC) {
            double rotX = x * Math.cos(-currentHeading) - y * Math.sin(-currentHeading);
            double rotY = x * Math.sin(-currentHeading) + y * Math.cos(-currentHeading);
            calculateMotorPowers(rotX, rotY, turn);
        }
        else {
            setMotorPowers(0, 0, 0, 0);
        }

    }

    private void calculateMotorPowers(double x, double y, double turn) {
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower  = power * cos/max + turn;
        double rightFrontPower = power * sin/max - turn;
        double leftBackPower   = power * sin/max + turn;
        double rightBackPower  = power * cos/max - turn;

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

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDriveMode(@NotNull DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }
}
