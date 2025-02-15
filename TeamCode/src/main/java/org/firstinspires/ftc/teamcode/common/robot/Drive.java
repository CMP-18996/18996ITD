package org.firstinspires.ftc.teamcode.common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;

@Config
public class Drive {
    private final HardwareMap hardwareMap;

    private final DcMotor leftFrontDrive;
    private final DcMotor leftBackDrive;
    private final DcMotor rightFrontDrive;
    private final DcMotor rightBackDrive;

    private DcMotor.ZeroPowerBehavior breakMode = DcMotor.ZeroPowerBehavior.BRAKE;

    public Drive(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        leftFrontDrive = hardwareMap.get(DcMotor.class, HardwareMapNames.LEFT_FRONT);
        leftBackDrive = hardwareMap.get(DcMotor.class, HardwareMapNames.LEFT_BACK);
        rightFrontDrive = hardwareMap.get(DcMotor.class, HardwareMapNames.RIGHT_FRONT);
        rightBackDrive = hardwareMap.get(DcMotor.class, HardwareMapNames.RIGHT_BACK);

        configureMotors();
    }

    public void robotCentricDrive(double x, double y, double turn) {
        calculateMotorPowers(x, y, turn);
    }

    private void calculateMotorPowers(double x, double y, double turn) {
        /*
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
         */

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
        double leftFrontPower = (y + x + turn) / denominator;
        double leftBackPower = (y - x + turn) / denominator;
        double rightFrontPower = (y - x - turn) / denominator;
        double rightBackPower = (y + x - turn) / denominator;


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

        leftFrontDrive.setZeroPowerBehavior(breakMode);
        leftBackDrive.setZeroPowerBehavior(breakMode);
        rightFrontDrive.setZeroPowerBehavior(breakMode);
        rightBackDrive.setZeroPowerBehavior(breakMode);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setBreakMode(@NotNull DcMotor.ZeroPowerBehavior breakMode) {
        this.breakMode = breakMode;

        leftFrontDrive.setZeroPowerBehavior(breakMode);
        leftBackDrive.setZeroPowerBehavior(breakMode);
        rightFrontDrive.setZeroPowerBehavior(breakMode);
        rightBackDrive.setZeroPowerBehavior(breakMode);

    }

    public DcMotor.ZeroPowerBehavior getBrakeMode() {
        return breakMode;
    }
}
