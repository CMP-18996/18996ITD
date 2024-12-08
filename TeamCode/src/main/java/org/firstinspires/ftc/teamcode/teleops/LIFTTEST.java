package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@TeleOp(name = "Intake Test")
public class LIFTTEST extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo servo = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_ROTATOR);
        Servo servo1 = hardwareMap.get(Servo.class, HardwareMapNames.BUCKET_SERVO);
        CRServo roller = hardwareMap.get(CRServo.class, HardwareMapNames.INTAKE_SERVO_1);

        waitForStart();

        while (opModeIsActive()) {
            double intake = -gamepad1.right_stick_y;
            double bucket = -gamepad1.left_stick_y;
            servo.setPosition(intake);
            servo1.setPosition(bucket);

            roller.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            telemetry.addData("Intake", intake);
            telemetry.addData("Bucket", intake);
            telemetry.update();
        }
    }
}
