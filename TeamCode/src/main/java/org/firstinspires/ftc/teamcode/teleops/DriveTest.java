package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.robot.Drive;
import org.firstinspires.ftc.teamcode.common.odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.common.odo.OdometryHardware;

@TeleOp(name = "Drive Test")
@Config
public class DriveTest extends LinearOpMode {

    public Drive drive;
    public OdometryHardware odometryHardware;

    public void runOpMode() {

        drive = new Drive(hardwareMap);
        drive.setDriveMode(Drive.DriveMode.FIELD_CENTRIC);

        odometryHardware = new OdometryHardware(hardwareMap);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            odometryHardware.pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
            double heading = odometryHardware.pinpoint.getHeading();
            drive.vectorDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, heading);

            if (gamepad1.circle) {
                odometryHardware.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
                telemetry.addLine("Reset Heading");
            }

            telemetry.addData("HEADING", heading);
            telemetry.update();
        }
    }
}
