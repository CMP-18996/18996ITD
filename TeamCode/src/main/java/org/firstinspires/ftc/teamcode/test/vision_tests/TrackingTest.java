package org.firstinspires.ftc.teamcode.test.vision_tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.odo.OdometryFusion;
import org.firstinspires.ftc.teamcode.odo.OdometryHardware;

@TeleOp(name = "BucketVisionTest", group = "Vision")
@Config
public class TrackingTest extends LinearOpMode {

    OdometryHardware odometryHardware;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode() {

        odometryHardware = new OdometryHardware(hardwareMap);

        PIDController pid = new PIDController(kP, kI, kD);

        pid.setSetPoint(0);

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.triangle) {
                pid.setPID(kP, kI, kD);

                telemetry.addLine("Set new PID constants!");
            }

            // Get the degrees of the april tag
            LLResult limelightResult = odometryHardware.limelight.getLatestResult();
            double aprilTagYaw = limelightResult.getTx();

            double output = pid.calculate(aprilTagYaw); // Calculate PID output

            double max;

            double leftFrontPower  = output;
            double rightFrontPower = -output;
            double leftBackPower   = output;
            double rightBackPower  = -output;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData("tx", aprilTagYaw);
            telemetry.addData("PID output", output);

            telemetry.update();
        }
    }
}
