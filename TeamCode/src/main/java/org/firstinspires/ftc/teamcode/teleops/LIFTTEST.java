package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;
import org.firstinspires.ftc.teamcode.common.robot.OdometryHardware;

@TeleOp(name = "Wrist Test")
public class LIFTTEST extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo servo = hardwareMap.get(Servo.class, HardwareMapNames.WRIST_SERVO);

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(gamepad1.right_stick_x);
        }
    }
}
