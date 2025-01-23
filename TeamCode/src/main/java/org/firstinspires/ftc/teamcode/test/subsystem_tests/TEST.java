package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@Config
@TeleOp
public class TEST extends LinearOpMode {

    public static double INTAKE_ROTATION_TRANSFER = 0.0; // max and min rotation used as what arm is actually being rotated to, subject to change
    public static double INTAKE_ROTATION_PICK_UP = 0.30;
    public static double INTAKE_ROTATION_MOVING = 0.25;

    public static double INTAKE_PIVOT_TRANSFER = 0.15;
    public static double INTAKE_PIVOT_PICK_UP = 0.30;
    public static double INTAKE_PIVOT_MOVING = 0.25;

    @Override
    public void runOpMode() {
        Servo bottom = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_BOTTOM_PIVOT);
        Servo top = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_TOP_PIVOT);
        CRServo intake = hardwareMap.get(CRServo.class, HardwareMapNames.INTAKE_ROLLER_SERVO);

        waitForStart();
        while(opModeIsActive()) {
            bottom.setPosition(INTAKE_ROTATION_TRANSFER);
            top.setPosition(INTAKE_PIVOT_TRANSFER);
            intake.setPower(0.2);
            sleep(1000);
            while (!gamepad1.a && !opModeIsActive()) ;
            {
            }
            bottom.setPosition(INTAKE_ROTATION_MOVING);
            top.setPosition(INTAKE_PIVOT_MOVING);
            sleep(1000);
            while (!gamepad1.a && !opModeIsActive()) ;
            {
            }
            bottom.setPosition(INTAKE_ROTATION_PICK_UP);
            top.setPosition(INTAKE_PIVOT_PICK_UP);
            intake.setPower(1);
            sleep(1000);
            while (!gamepad1.a && !opModeIsActive()) ;
            {
            }
        }
    }
}
