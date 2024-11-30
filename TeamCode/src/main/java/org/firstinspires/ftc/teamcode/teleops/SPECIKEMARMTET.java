package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.robot.Drive;
import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@TeleOp(name = "!ARM TEST")
@Config
public class SPECIKEMARMTET extends LinearOpMode {

    private DcMotorEx arm1;
    private Servo arm2;

    private int arm1_position = 0;

    public static double Kp = 0.008;
    public static double Kd = -0.0007;
    public static double Ki = 0.02;

    public static double Kg = 0.2;

    public static double MAX_EXTENSION_SPEED = 1.0;
    public static double MAX_RETURN_SPEED = 1.0;

    public static double SERVO = 0.6;

    Drive drive;

    ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() {

        arm1 = hardwareMap.get(DcMotorEx.class, HardwareMapNames.ARM_MOTOR);
        arm2 = hardwareMap.get(Servo.class, HardwareMapNames.WRIST_SERVO);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm1.setDirection(DcMotorSimple.Direction.FORWARD);
        arm2.setDirection(Servo.Direction.FORWARD);

        drive = new Drive(hardwareMap);
        drive.setBreakMode(DcMotor.ZeroPowerBehavior.BRAKE);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        int lastError = 0;

        double integralSum = 0;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.circle && !previousGamepad1.circle) {
                //ARM_ADDITION = 0.0;
                arm1_position = 550;
            }
            else if (currentGamepad1.square && !previousGamepad1.square) {
                arm1_position = 50;
                //ARM_ADDITION = 0.0;
            }
            else if (currentGamepad1.triangle && !previousGamepad1.triangle) {
                arm1_position = 100;
                //ARM_ADDITION = 0.0;
            }

            arm1_position += (int) ((currentGamepad1.right_trigger - currentGamepad1.left_trigger) * 10);

            double angleFromTicks = -( (360 * (arm1.getCurrentPosition())) /1503.6) - 2;
            double G = -Kg * Math.cos(Math.toRadians(angleFromTicks));

            int error = arm1_position + arm1.getCurrentPosition();
            double P = Kp * -error;

            double D = Kd * (error - lastError) / timer.seconds();

            integralSum = integralSum + (error * timer.seconds());
            double I = Ki * -integralSum;

            if (angleFromTicks > 90) {
                I = 0;
            }


            lastError = error;
            timer.reset();

            double power = Range.clip(P + D + I + G, -MAX_EXTENSION_SPEED, MAX_RETURN_SPEED);
            arm1.setPower(power);

            arm2.setPosition(SERVO);

            telemetry.addData("SETPOINT", arm1_position);
            telemetry.addData("POSITION", arm1.getCurrentPosition());
            telemetry.addData("ERROR", error);
            telemetry.addData("ANGLE FROM TICKS", angleFromTicks);

            TelemetryPacket packet = new TelemetryPacket();

            packet.put("P", P);
            packet.put("I", I);
            packet.put("D", D);
            packet.put("G", G);
            packet.put("ERROR", error);
            packet.put("POWER", power);
            packet.put("POSITION", arm1_position);

            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();

            drive.robotCentricDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}
