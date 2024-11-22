package org.firstinspires.ftc.teamcode.test.vision_tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.robot.OdometryHardware;

@TeleOp(name = "BucketVisionTest", group = "Vision")
@Disabled
public class TrackingTest extends LinearOpMode {

    OdometryHardware odometryHardware;

    public static double kP = 0.01;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        odometryHardware = new OdometryHardware(hardwareMap);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "lF");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lB");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rF");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rB");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            // Get the degrees of the april tag
            LLResult limelightResult = odometryHardware.limelight.getLatestResult();
            double aprilTagYaw = limelightResult.getTx() + 0.1;

            double max;

            double leftFrontPower  = aprilTagYaw * kP;
            double rightFrontPower = -aprilTagYaw * kP;
            double leftBackPower   = aprilTagYaw * kP;
            double rightBackPower  = -aprilTagYaw * kP;

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
            telemetry.addData("output power", aprilTagYaw * kP);

            telemetry.update();
        }
    }
}
