package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.robot.Drive;
import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@TeleOp(name = "Drive Test")
public class DriveTest extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() {
        follower = new Follower(hardwareMap);
        follower.startTeleopDrive();

        waitForStart();

        while(opModeIsActive()) {
            double xVal = gamepad1.left_stick_x;
            double yVal = -gamepad1.left_stick_y;
            double hVal = gamepad1.right_stick_x;

            double xSetRate = Drive.calculateXRate(xVal);
            double ySetRate = Drive.calculateYRate(yVal);
            double hSetRate = Drive.calculateHRate(hVal);

            double xCurrentRate = follower.getVelocity().getXComponent();
            double yCurrentRate = follower.getVelocity().getYComponent();
            double hCurrentRate = follower.poseUpdater.getAngularVelocity();

            double xVector = Drive.calculateXVectorComponent(xCurrentRate, xSetRate);
            double yVector = Drive.calculateYVectorComponent(yCurrentRate, ySetRate);
            double hVector = Drive.calculateHVectorComponent(hCurrentRate, hSetRate);

            follower.setTeleOpMovementVectors(xVector, yVector, hVector, true);

            telemetry.addData("xVal", xVal);
            telemetry.addData("yVal", yVal);
            telemetry.addData("hVal", hVal);

            telemetry.addData("xSet", xSetRate);
            telemetry.addData("ySet", ySetRate);
            telemetry.addData("hSet", hSetRate);

            telemetry.addData("xCurr", xCurrentRate);
            telemetry.addData("yCurr", yCurrentRate);
            telemetry.addData("hCurr", hCurrentRate);

            telemetry.addData("xVector", xVector);
            telemetry.addData("yVector", yVector);
            telemetry.addData("hVector", hVector);

            telemetry.update();
        }
    }
}

