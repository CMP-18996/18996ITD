package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.robot.Drive;
import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Disabled
@TeleOp(name = "Drive Test")
public class DriveTest extends LinearOpMode {

    private Follower follower;

    public double xVector = 0;
    public double yVector = 0;
    public double hVector = 0;

    @Override
    public void runOpMode() {
        Constants.setConstants(FConstants.class, LConstants.class);

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

            double xCurrentRate = follower.poseUpdater.getVelocity().getXComponent();
            double yCurrentRate = follower.poseUpdater.getVelocity().getYComponent();
            double hCurrentRate = follower.poseUpdater.getAngularVelocity();

            double xDelta = Drive.calculateXVectorDelta(xCurrentRate, xSetRate);
            double yDelta = Drive.calculateYVectorDelta(yCurrentRate, ySetRate);
            double hDelta = Drive.calculateHVectorDelta(hCurrentRate, hSetRate);

            xVector += xDelta;
            yVector += yDelta;
            hVector += hDelta;

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

            telemetry.addData("xDelta", xDelta);
            telemetry.addData("yDelta", yDelta);
            telemetry.addData("hDelta", hDelta);

            telemetry.addData("xVector", xVector);
            telemetry.addData("yVector", yVector);
            telemetry.addData("hVector", hVector);

            telemetry.update();
        }
    }
}

