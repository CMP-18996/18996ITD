package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.robot.Drive;
import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;
import org.firstinspires.ftc.teamcode.common.odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.common.odo.OdometryHardware;

@TeleOp(name = "HardwareTeleop")
@Config
public class HardwareTeleop extends LinearOpMode {

    public static double INTAKE_SERVO_SPEED = -1.0;

    public static double INTAKE_ROTATOR_ACTIVE = 0.5;
    public static double INTAKE_ROTATOR_INACTIVE = 0.0;
    public static double INTAKE_ROTATOR_OFFSET = 0.05;

    public static double BUCKET_SERVO_IDLE = 0.0;
    public static double BUCKET_SERVO_HALF = 0.05;
    public static double BUCKET_SERVO_FULL = 0.13;

    public static double SPECIMEN_OPEN = 0.0;
    public static double SPECIMEN_CLOSED = 1.0;

    //public static int LIFT_ZERO_POSITION = 0;
    public static int LIFT_LOW_CHAMBER = 250; //idfk
    public static int LIFT_HIGH_CHAMBER = 650; //idfk
    public static int LIFT_LOW_BUCKET = 450;
    public static int LIFT_HIGH_BUCKET = 850;
    // 100

    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.000;
    public static double kF = 0.0;
    public static double LIFT_TOLERANCE = 10;

    public enum COLOR {
        RED,
        YELLOW,
        BLUE;
    }

    private ElapsedTime runtime = new ElapsedTime();

    private Drive drive;
    private OdometryHardware odometryHardware;

    public void runOpMode() throws InterruptedException{
        Gamepad currentGamepad1 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();

        drive = new Drive(hardwareMap);
        drive.setDriveMode(Drive.DriveMode.FIELD_CENTRIC);

        odometryHardware = new OdometryHardware(hardwareMap);

        boolean intakeActive = false;

        boolean bucketActive = false;

        int liftState = 0;

        double liftPower = 0;

        double rotatorOffset = 0;

        double turnOffTime = 0;

        //int liftTarget = 0;

        Servo intakeRotator = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_ROTATOR);
        intakeRotator.scaleRange(0.0, 1.0);
        intakeRotator.setDirection(Servo.Direction.REVERSE);

        CRServo intake1 = hardwareMap.get(CRServo.class, HardwareMapNames.INTAKE_SERVO_1);
        CRServo intake2 = hardwareMap.get(CRServo.class, HardwareMapNames.INTAKE_SERVO_2);
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, HardwareMapNames.INTAKE_COLOR_SENSOR);

        DcMotorEx extension = hardwareMap.get(DcMotorEx.class, HardwareMapNames.EXTENSION_MOTOR);
        //extension.setMode(RUN_USING_ENCODER);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, HardwareMapNames.LIFT_MOTOR);
        //liftMotor.setTargetPosition(0);
        //liftMotor.setMode(RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFController liftPIDF = new PIDFController(kP, kI, kD, kF);
        liftPIDF.setTolerance(LIFT_TOLERANCE);

        Servo bucketServo = hardwareMap.get(Servo.class, HardwareMapNames.BUCKET_SERVO);
        bucketServo.scaleRange(0.0, 1.0);

        DcMotorEx hang = hardwareMap.get(DcMotorEx.class, HardwareMapNames.HANG_MOTOR_1);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo specimenServo = hardwareMap.get(Servo.class, HardwareMapNames.SPECIMEN_SERVO);
        specimenServo.scaleRange(0.6, 0.8);

        bucketServo.setPosition(BUCKET_SERVO_IDLE);
        specimenServo.setPosition(SPECIMEN_CLOSED);
        intakeRotator.setPosition(INTAKE_ROTATOR_INACTIVE);

        intake1.setPower(0);
        intake2.setPower(0);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            // COLOR
            double r, g, b;
            r = colorSensor.red();
            g = colorSensor.green();
            b = colorSensor.blue();

            COLOR color;

            if (colorSensor.alpha() > 100) {
                if (r > g && r > b) {
                    color = COLOR.RED;
                } else if (g > r && g > b) {
                    color = COLOR.YELLOW;
                } else if (b > g && b > r) {
                    color = COLOR.BLUE;
                }
            }
            /*
            YELLOW
            402
            544
            125
            359

            RED
            253
            140
            66
            157

            BLUE
            72
            152
            335
            186
             */

            //
            // INTAKE
            //
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                intakeActive = !intakeActive;

                if (!intakeActive) {
                    turnOffTime = runtime.milliseconds() + 1000;
                }
            }

            //if (color = COLOR.GREEN)

            if (intakeActive) {
                rotatorOffset = -currentGamepad1.right_stick_y * INTAKE_ROTATOR_OFFSET;
                rotatorOffset = 0;
                intakeRotator.setPosition(INTAKE_ROTATOR_ACTIVE + rotatorOffset);

                if (currentGamepad1.left_bumper) {
                    intake1.setPower(-INTAKE_SERVO_SPEED);
                    intake2.setPower(-INTAKE_SERVO_SPEED);
                }
                else {
                    intake1.setPower(INTAKE_SERVO_SPEED);
                    intake2.setPower(INTAKE_SERVO_SPEED);
                }
            }
            else {
                if (currentGamepad1.left_bumper) {
                    intake1.setPower(-INTAKE_SERVO_SPEED);
                    intake2.setPower(-INTAKE_SERVO_SPEED);
                }
                else {
                    if (runtime.milliseconds() > turnOffTime) {
                        intake1.setPower(0);
                        intake2.setPower(0);
                    }
                    else {
                        intake1.setPower(INTAKE_SERVO_SPEED);
                        intake2.setPower(INTAKE_SERVO_SPEED);
                    }
                }

                intakeRotator.setPosition(INTAKE_ROTATOR_INACTIVE);
            }


            //
            // EXTENSION (cubic control)
            //
            extension.setPower(Math.pow(currentGamepad1.right_trigger - currentGamepad1.left_trigger, 3));

            //
            // LIFT
            //
            // lift
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                if (liftState == 4) {
                    liftState = 0;
                    liftPIDF.setSetPoint(0);
                    bucketServo.setPosition(BUCKET_SERVO_IDLE);
                }
                else {
                    liftState = 4;
                    liftPIDF.setSetPoint(LIFT_HIGH_BUCKET);
                    bucketServo.setPosition(BUCKET_SERVO_HALF);
                }
            }
            else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                if (liftState == 3) {
                    liftState = 0;
                    liftPIDF.setSetPoint(0);
                    bucketServo.setPosition(BUCKET_SERVO_IDLE);
                }
                else {
                    liftState = 3;
                    liftPIDF.setSetPoint(LIFT_HIGH_CHAMBER);
                    bucketServo.setPosition(BUCKET_SERVO_IDLE);
                }
            }
            else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                if (liftState == 2) {
                    liftState = 0;
                    liftPIDF.setSetPoint(0);
                    bucketServo.setPosition(BUCKET_SERVO_IDLE);
                }
                else {
                    liftState = 2;
                    liftPIDF.setSetPoint(LIFT_LOW_BUCKET);
                    bucketServo.setPosition(BUCKET_SERVO_HALF);
                }
            }
            else if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                if (liftState == 1) {
                    liftState = 0;
                    liftPIDF.setSetPoint(0);
                    bucketServo.setPosition(BUCKET_SERVO_IDLE);
                }
                else {
                    liftState = 1;
                    liftPIDF.setSetPoint(LIFT_LOW_CHAMBER);
                    bucketServo.setPosition(BUCKET_SERVO_IDLE);
                }
            }

            // lift PID
            liftPower = liftPIDF.calculate(-liftMotor.getCurrentPosition());
            if (Math.abs(liftPower) > 1) {
                liftPower /= Math.abs(liftPower);
                telemetry.addLine("EXCEED POWER !!!!");
            }
            liftMotor.setPower(-currentGamepad1.right_stick_y);

            if (currentGamepad1.dpad_up) {
                hang.setPower(1.0);
            }
            else if (currentGamepad1.dpad_down) {
                hang.setPower(-1.0);
            }
            else {
                hang.setPower(0);
            }




            // bucket
            if (currentGamepad1.cross && !previousGamepad1.cross) {
                if (liftState == 1 || liftState == 3) {
                    liftMotor.setTargetPosition(0);
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

            // UDPDATE
            if (currentGamepad1.triangle && !previousGamepad1.triangle) {
                liftPIDF.setPIDF(kP, kI, kD, kF);
                liftPIDF.setTolerance(LIFT_TOLERANCE);
                telemetry.addLine("UPDATED VALUES");
            }

            //
            // HANG
            //


            //
            // DRIVE
            //
            odometryHardware.pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
            double heading = odometryHardware.pinpoint.getHeading();
            drive.vectorDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, heading);

            if (gamepad1.options) {
                odometryHardware.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
                telemetry.addLine("Reset Heading");
            }


            telemetry.addData("INTAKE ACTIVE", intakeActive);
            telemetry.addData("INTAKE OFFSET", rotatorOffset);
            telemetry.addLine();

            telemetry.addData("BUCKET ACTIVE", bucketActive);
            telemetry.addLine();

            telemetry.addData("EXTENSION POS", extension.getCurrentPosition());
            telemetry.addLine();

            telemetry.addData("LIFT STATE", liftState);
            telemetry.addData("LIFT SET POINT", liftPIDF.getSetPoint());
            telemetry.addData("LIFT POS", liftMotor.getCurrentPosition());
            telemetry.addData("LIFT POWER", liftPower);
            telemetry.addLine();

            telemetry.addData("RED", colorSensor.red());
            telemetry.addData("GREEN", colorSensor.green());
            telemetry.addData("BLUE", colorSensor.blue());
            telemetry.addData("ALPHA", colorSensor.alpha());
            telemetry.addLine();

            telemetry.addData("HEADING", Math.toDegrees(heading));

            telemetry.update();
        }
    }
}
