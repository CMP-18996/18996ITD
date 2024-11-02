package org.firstinspires.ftc.teamcode.odo;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@TeleOp(name = "PPTest")
@Config
public class PPTestTeleop extends LinearOpMode {

    public Drive drive;
    public OdometryHardware odometryHardware;
    public OdometryFusion odometryFusion;

    public LinearPathing linearPathing;

    public void runOpMode() {

        // Create all odo stuff
        drive = new Drive(hardwareMap);
        odometryHardware = new OdometryHardware(hardwareMap);
        odometryFusion = new OdometryFusion(odometryHardware);
        linearPathing = new LinearPathing(drive, odometryFusion);

        waitForStart();

        while (opModeIsActive()) {
            linearPathing.pointToPointDrive(null);
        }
    }
}
