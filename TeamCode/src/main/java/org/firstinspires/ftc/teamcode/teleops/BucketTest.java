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

@TeleOp(name = "Bucket Test")
@Config
public class BucketTest extends LinearOpMode {

    public void runOpMode() {
        Gamepad currentGamepad1 = new Gamepad();

        Servo bucketServo = hardwareMap.get(Servo.class, HardwareMapNames.BUCKET_SERVO);
        bucketServo.scaleRange(0.0, 1.0);
        bucketServo.setDirection(Servo.Direction.FORWARD);

        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, HardwareMapNames.LIFT_MOTOR);
        liftMotor.setMode(STOP_AND_RESET_ENCODER);
        liftMotor.setMode(RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            currentGamepad1.copy(gamepad1);

            bucketServo.setPosition(currentGamepad1.right_trigger);

            liftMotor.setPower(-currentGamepad1.right_stick_y);

            telemetry.addData("LIFT", liftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
