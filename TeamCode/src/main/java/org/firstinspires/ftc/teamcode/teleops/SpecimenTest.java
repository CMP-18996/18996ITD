package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.equation.IntegerSequence;
import org.firstinspires.ftc.teamcode.common.odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.common.robot.Drive;
import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;
import org.firstinspires.ftc.teamcode.common.robot.OdometryHardware;

@TeleOp(name = "Specimen Arm Test")
@Config
public class SpecimenTest extends LinearOpMode {

    private DcMotorEx arm1;
    private Servo arm2;
    private Servo armClaw;

    private int arm1_position = 0;
    private double arm2_position = 0.87;

    public static int DEPOSIT_1 = 540;
    public static double DEPOSIT_2 = 0.4;

    public static int GRAB_1 = 30;
    public static double GRAB_2 = 0.73;

    public static int REST_1 = 0;
    public static double REST_2 = 0.87;

    public static double CLAW_OPEN = 0.5;
    public static double CLAW_CLOSED = 0.82;

    public static double Kp = 0.0303;
    public static double Kd = 0.0001;
    public static double Ki = 0.0;
    public static double Kg = 0.2;

    Drive drive;

    ElapsedTime elapsedTime = new ElapsedTime();

    PIDController pidController = new PIDController(Kp, Ki, Kd);

    @Override
    public void runOpMode() {

        arm1 = hardwareMap.get(DcMotorEx.class, HardwareMapNames.ARM_MOTOR);
        arm2 = hardwareMap.get(Servo.class, HardwareMapNames.WRIST_SERVO);
        armClaw = hardwareMap.get(Servo.class, HardwareMapNames.GRIPPER_SERVO);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm1.setDirection(DcMotorSimple.Direction.FORWARD);
        arm2.setDirection(Servo.Direction.FORWARD);
        armClaw.setDirection(Servo.Direction.FORWARD);

        drive = new Drive(hardwareMap);
        drive.setBreakMode(DcMotor.ZeroPowerBehavior.BRAKE);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        boolean clawClosed = false;

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            //
            // STATE CONTROL
            //

            if (currentGamepad1.square) {
                arm1_position = GRAB_1;
                arm2_position = GRAB_2;
            }
            else if (currentGamepad1.circle) {
                arm1_position = DEPOSIT_1;
                arm2_position = DEPOSIT_2;
            }
            else if (currentGamepad1.triangle) {
                arm1_position = REST_1;
                arm2_position = REST_2;
            }

            if (currentGamepad1.cross && !previousGamepad1.cross) {
                clawClosed = !clawClosed;
            }


            //
            // GUIDED CONTROL
            //
            double inverse = gamepad1.right_trigger - gamepad1.left_trigger;



            //
            // MANUAL CONTROL
            //
            //double arm1Delta = -Math.pow(currentGamepad1.right_stick_y, 3)/10;
            double arm2Delta = -Math.pow(currentGamepad1.left_stick_y, 1)/10;

            //arm1_position += arm1Delta;
            arm2_position += arm2Delta;

            /*
            if (arm1_position < 0.0) {
                arm1_position = 0;
            }
            if (arm1_position > 1.0) {
                arm1_position = 1.0;
            }
            */

            if (arm2_position < 0.0) {
                arm2_position = 0.0;
            }
            if (arm2_position > 1.0) {
                arm2_position = 1.0;
            }



            arm2.setPosition(arm2_position);

            if (clawClosed) {
                armClaw.setPosition(CLAW_CLOSED);
            }
            else {
                armClaw.setPosition(CLAW_OPEN);
            }

            if (currentGamepad1.dpad_up) {
                pidController.setPID(Kp, Ki, Kd);
                telemetry.addData("GUYGRAEHJBKNFEAGE", "W^T&AUYRGIAHGJWAKLG");
            }



            double angleFromTicks = ( (360 * (arm1.getCurrentPosition())) /1503.6);
            double G = Kg * Math.cos(Math.toRadians(angleFromTicks + 180 + 4));

            double PID = pidController.calculate(arm1.getCurrentPosition(), arm1_position);

            int error = arm1_position + arm1.getCurrentPosition();

            double power = PID + G;
            arm1.setPower(power);

            telemetry.addData("CLAW", clawClosed);
            telemetry.addData("SERVOPOS", arm2_position);
            telemetry.addData("DELTA 2", arm2Delta);
            telemetry.addData("", "");
            telemetry.addData("SETPOINT", arm1_position);
            telemetry.addData("POSITION", arm1.getCurrentPosition());
            telemetry.addData("ERROR", error);
            telemetry.addData("P", PID);
            telemetry.addData("", "");
            telemetry.addData("ANGLE FROM TICKS", angleFromTicks);
            telemetry.addData("G", G);
            telemetry.addData("", "");
            telemetry.addData("POWER", power);

            telemetry.update();

            drive.robotCentricDrive(gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x);
        }
    }
}
