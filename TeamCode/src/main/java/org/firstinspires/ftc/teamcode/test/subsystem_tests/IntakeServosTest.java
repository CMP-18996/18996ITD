package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@TeleOp(name = "Intake Servos Test")
public class IntakeServosTest extends LinearOpMode {

    private Servo intakeArm = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_BOTTOM_PIVOT);
    private Servo intakeWrist = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_TOP_PIVOT);

    @Override
    public void runOpMode() {

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a) {
                intakeArm.setPosition(1.0);
                intakeWrist.setPosition(1.0);
            }
            else {
                intakeArm.setPosition(0.0);
                intakeWrist.setPosition(0.0);
            }
            telemetry.addData("GHEIAUYHTEAUWFNIUAF|", 0);
            telemetry.update();
        }
    }
}

