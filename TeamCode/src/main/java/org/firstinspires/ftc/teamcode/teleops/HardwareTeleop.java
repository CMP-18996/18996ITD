package org.firstinspires.ftc.teamcode.teleops;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@TeleOp(name = "HardwareTeleop")
@Config
public class HardwareTeleop extends LinearOpMode {

    public static double INTAKE_SERVO_SPEED = 1.0;

    public static double INTAKE_ROTATOR_ACTIVE = 1.0;
    public static double INTAKE_ROTATOR_INACTIVE = 0.0;

    public static double EXTENSION_MOTOR_MAX_TPS = 1000;

    public static double BUCKET_SERVO_IDLE = 0.0;
    public static double BUCKET_SERVO_HALF = 0.5;
    public static double BUCKET_SERVO_FULL = 1.0;

    public static double SPECIMEN_OPEN = 0.0;
    public static double SPECIMEN_CLOSED = 0.0;

    public static int LIFT_ZERO_POSITION = 0;
    public static int LIFT_LOW_CHAMBER = 500; //idfk
    public static int LIFT_HIGH_CHAMBER = 1500; //idfk
    public static int LIFT_LOW_BUCKET = 1000;
    public static int LIFT_HIGH_BUCKET = 2000;

    public void runOpMode() {
        Gamepad currentGamepad1 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();

        boolean intakeActive = false;

        boolean bucketActive = false;

        int liftState = 0;

        Servo intakeRotator = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_ROTATOR);

        CRServo intake1 = hardwareMap.get(CRServo.class, HardwareMapNames.INTAKE_SERVO_1);
        CRServo intake2 = hardwareMap.get(CRServo.class, HardwareMapNames.INTAKE_SERVO_2);

        DcMotorEx extension = hardwareMap.get(DcMotorEx.class, HardwareMapNames.EXTENSION_MOTOR);
        extension.setMode(RUN_USING_ENCODER);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, HardwareMapNames.LIFT_MOTOR);
        liftMotor.setMode(RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo bucketServo = hardwareMap.get(Servo.class, HardwareMapNames.BUCKET_SERVO);

        Servo specimenServo = hardwareMap.get(Servo.class, HardwareMapNames.SPECIMEN_SERVO);

        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, HardwareMapNames.LEFT_FRONT);
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, HardwareMapNames.LEFT_BACK);
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, HardwareMapNames.RIGHT_FRONT);
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, HardwareMapNames.LEFT_BACK);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            //
            // INTAKE
            //
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                intakeActive = !intakeActive;
            }

            if (intakeActive) {
                intake1.setPower(INTAKE_SERVO_SPEED);
                intake2.setPower(INTAKE_SERVO_SPEED);

                intakeRotator.setPosition(INTAKE_ROTATOR_ACTIVE);
            }
            else {
                intake1.setPower(0);
                intake2.setPower(0);

                intakeRotator.setPosition(INTAKE_ROTATOR_INACTIVE);
            }

            if (currentGamepad1.left_bumper) {
                intake1.setPower(-1.0);
                intake2.setPower(-1.0);
            }

            //
            // EXTENSION
            //
            extension.setVelocity(EXTENSION_MOTOR_MAX_TPS *
                    (currentGamepad1.right_trigger - currentGamepad1.left_trigger));

            //
            // LIFT
            //
            // lift
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                if (liftState == 4) {
                    liftState = 0;
                    liftMotor.setTargetPosition(LIFT_ZERO_POSITION);
                    bucketServo.setPosition(BUCKET_SERVO_IDLE);
                }
                else {
                    liftState = 4;
                    liftMotor.setTargetPosition(LIFT_HIGH_BUCKET);
                    bucketServo.setPosition(BUCKET_SERVO_HALF);
                }
            }
            else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                if (liftState == 3) {
                    liftState = 0;
                    liftMotor.setTargetPosition(LIFT_ZERO_POSITION);
                    bucketServo.setPosition(BUCKET_SERVO_IDLE);
                }
                else {
                    liftState = 3;
                    liftMotor.setTargetPosition(LIFT_HIGH_CHAMBER);
                    bucketServo.setPosition(BUCKET_SERVO_IDLE);
                }
            }
            else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                if (liftState == 2) {
                    liftState = 0;
                    liftMotor.setTargetPosition(LIFT_ZERO_POSITION);
                    bucketServo.setPosition(BUCKET_SERVO_IDLE);
                }
                else {
                    liftState = 2;
                    liftMotor.setTargetPosition(LIFT_LOW_BUCKET);
                    bucketServo.setPosition(BUCKET_SERVO_HALF);
                }
            }
            else if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                if (liftState == 1) {
                    liftState = 0;
                    liftMotor.setTargetPosition(LIFT_ZERO_POSITION);
                    bucketServo.setPosition(BUCKET_SERVO_IDLE);
                }
                else {
                    liftState = 1;
                    liftMotor.setTargetPosition(LIFT_LOW_CHAMBER);
                    bucketServo.setPosition(BUCKET_SERVO_IDLE);
                }
            }

            //bucket
            if (currentGamepad1.cross && !previousGamepad1.cross) {
                if (liftState == 1 || liftState == 3) {
                    liftMotor.setTargetPosition(LIFT_ZERO_POSITION);
                }
                else {
                    bucketActive = !bucketActive;
                    if (bucketActive) {
                        bucketServo.setPosition(BUCKET_SERVO_FULL);
                    } else {
                        bucketServo.setPosition(BUCKET_SERVO_IDLE);
                    }
                }
            }

            // specimen
            if (currentGamepad1.square && !previousGamepad1.square) {
                specimenServo.setPosition(SPECIMEN_CLOSED);
            }
            else if (currentGamepad1.circle && !previousGamepad1.circle) {
                specimenServo.setPosition(SPECIMEN_OPEN);
            }

            //
            // HANG
            //


            //
            // DRIVE
            //
            double x =  currentGamepad1.left_stick_x;
            double y   = -currentGamepad1.left_stick_y;
            double turn =  currentGamepad1.right_stick_x;

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

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData("INTAKE ACTIVE", intakeActive);
            telemetry.addData("BUCKET ACTIVE", bucketActive);
            telemetry.addData("LIFT STATE", liftState);
            telemetry.update();
        }
    }
}
