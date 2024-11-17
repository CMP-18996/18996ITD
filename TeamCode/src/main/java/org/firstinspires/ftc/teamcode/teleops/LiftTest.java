package org.firstinspires.ftc.teamcode.teleops;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

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

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@TeleOp(name = "Lift Test")
@Config
public class LiftTest extends LinearOpMode {

    public static double kP = 0.001;

    public static double currTarget = 0;


    public void runOpMode() {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, HardwareMapNames.LIFT_MOTOR);
        liftMotor.setMode(STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setTargetPositionTolerance(10);
        liftMotor.setMode(RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                currTarget += 100;
            }
            else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                currTarget -= 100;
            }

            int currPosition = liftMotor.getCurrentPosition();

            double motorPower = kP * (currTarget - currPosition);
            liftMotor.setPower(motorPower);

            telemetry.addData("LIFT POS", currPosition);
            telemetry.addData("TARGET", currTarget);
            telemetry.addData("POWER", motorPower);
            telemetry.update();
        }
    }
}
