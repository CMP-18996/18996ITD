package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.ejml.equation.IntegerSequence;
import org.firstinspires.ftc.teamcode.common.odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.common.robot.Drive;
import org.firstinspires.ftc.teamcode.common.robot.OdometryHardware;

@TeleOp(name = "Specimen Arm Test")
@Config
public class SpecimenTest extends LinearOpMode {
    private Servo arm1;
    private Servo arm2;
    private Servo armClaw;

    private double arm1_position = 0.0;
    private double arm2_position = 0.0;

    public static double DEPOSIT_1 = 0.3;
    public static double DEPOSIT_2 = 0.3;

    public static double GRAB_1 = 0.79;
    public static double GRAB_2 = 0.73;

    public static double REST_1 = 0.5;
    public static double REST_2 = 0.5;

    public static double CLAW_OPEN = 0.0;
    public static double CLAW_CLOSED = 0.85;

    Drive drive;

    @Override
    public void runOpMode() {

        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        armClaw = hardwareMap.get(Servo.class, "armClaw");

        arm1.setDirection(Servo.Direction.FORWARD);
        arm2.setDirection(Servo.Direction.FORWARD);
        armClaw.setDirection(Servo.Direction.FORWARD);

        arm1.scaleRange(0.0, 1.0);
        arm2.scaleRange(0.0, 1.0);
        armClaw.scaleRange(0.0, 1.0);

        drive = new Drive(hardwareMap);

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
            /*
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
            */

            //
            // GUIDED CONTROL
            //
            double inverse = gamepad1.right_trigger - gamepad1.left_trigger;



            //
            // MANUAL CONTROL
            //
            double arm1Delta = -Math.pow(currentGamepad1.right_stick_y, 2)/20;
            double arm2Delta = -Math.pow(currentGamepad1.left_stick_y, 2)/20;

            arm1_position += arm1Delta;
            arm2_position += arm2Delta;

            if (arm1_position < 0.0) {
                arm1_position = 0;
            }
            if (arm1_position > 1.0) {
                arm1_position = 1.0;
            }

            if (arm2_position < 0.0) {
                arm2_position = 0.0;
            }
            if (arm2_position > 1.0) {
                arm2_position = 1.0;
            }


            //
            // waqsedxrfokpjihugyftdreswardjihugyftdrseawsrdtyuijoh
            //

            if (clawClosed) {
                armClaw.setPosition(CLAW_CLOSED);
            }
            else {
                armClaw.setPosition(CLAW_OPEN);
            }


            telemetry.addData("CLAW", clawClosed);
            telemetry.addData("ARM1", arm1_position);
            telemetry.addData("ARM2", arm2_position);
            telemetry.addData("DELTA 1", arm1Delta);
            telemetry.addData("DELTA 2", arm2Delta);

            telemetry.update();

            drive.robotCentricDrive(gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x);

            arm1.setPosition(arm1_position);
            arm2.setPosition(arm2_position);
        }
    }
}
