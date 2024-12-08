package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.common.robot.Drive;
import org.firstinspires.ftc.teamcode.common.robot.OdometryHardware;

@TeleOp(name = "!Drive Test")
public class DriveTesting extends LinearOpMode {

    private OdometryHardware odometryHardware;
    private Drive drive;

    @Override
    public void runOpMode() {

        odometryHardware = new OdometryHardware(hardwareMap);
        drive = new Drive(hardwareMap);
        drive.setBreakMode(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.setAutoAlignMode(Drive.AutoAlignMode.NONE);

        odometryHardware.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.cross) {
                drive.setAutoAlignMode(Drive.AutoAlignMode.WALL);
            }
            if (gamepad1.square) {
                drive.setAutoAlignMode(Drive.AutoAlignMode.NONE);
            }

            odometryHardware.pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
            double heading = odometryHardware.pinpoint.getHeading();

            drive.robotCentricDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, heading);

            telemetry.addData("TURN", drive.TURN_);
            telemetry.update();
        }
    }
}
