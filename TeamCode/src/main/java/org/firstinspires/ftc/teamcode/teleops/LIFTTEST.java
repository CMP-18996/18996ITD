package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@TeleOp(name = "!CLAW Test")
public class LIFTTEST extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo claw = hardwareMap.get(Servo.class, "claw");

        waitForStart();

        while (opModeIsActive()) {

            claw.setPosition(gamepad1.right_trigger);

            telemetry.addData("VALUE:", gamepad1.right_trigger);
            telemetry.update();
        }
    }
}
