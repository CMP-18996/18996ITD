package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@Disabled
@Config
@TeleOp(name = "TRANSFER")
public class TransferTesting extends LinearOpMode{

    public static double transferServoValue = 0.25;
    public static double bucketTransfer = 0.0;

    public static double clawCLosed = 0.35;
    public static double clawOpen = 0.8;

    @Override
    public void runOpMode() {

        Servo intakeRotator = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_ROTATOR);
        Servo bucketServo = hardwareMap.get(Servo.class, HardwareMapNames.BUCKET_SERVO);
        CRServo intake = hardwareMap.get(CRServo.class, HardwareMapNames.INTAKE_SERVO_1);
        Servo clawServo = hardwareMap.get(Servo.class, "claw");

        waitForStart();

        while (opModeIsActive()) {

            intake.setPower(1.0);
            intakeRotator.setPosition(gamepad1.right_trigger);
        }
    }
}
