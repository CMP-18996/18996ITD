package org.firstinspires.ftc.teamcode.common.odo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/* Op mode for testing OdometryHardware and Odometry Fusion classes */
@TeleOp(name="OdoTest")
public class OdoTest extends LinearOpMode {

    OdometryHardware odometryHardware;
    OdometryFusion odometryFusion;

    public static double otos_linear_weight = 1;
    public static double otos_angular_weight = 1;

    public static double pinpoint_linear_weight = 1;
    public static double pinpoint_angular_weight = 1;

    public static double limelight_linear_weight = 1;

    @Override
    public void runOpMode() {
        odometryHardware = new OdometryHardware(hardwareMap);

        odometryFusion = new OdometryFusion(odometryHardware);

        OdometryFusion.setLinearWeights(otos_linear_weight, pinpoint_linear_weight, limelight_linear_weight);
        OdometryFusion.setAngularWeights(otos_angular_weight, pinpoint_angular_weight);

        waitForStart();

        while (opModeIsActive()) {
            OdometryHardware.MultiPose2D posPos2D = odometryHardware.getPosData(true, true, true);
            OdometryHardware.MultiPose2D velPos2D = odometryHardware.getAllVelData();

            odometryFusion.updateOdometryData();

            telemetry.addData("OTOS X COORD", posPos2D.otosPos.getX(DistanceUnit.INCH));
            telemetry.addData("OTOS Y COORD", posPos2D.otosPos.getY(DistanceUnit.INCH));
            telemetry.addData("OTOS H COORD", posPos2D.otosPos.getHeading(AngleUnit.DEGREES));

            telemetry.addData("PINPOINT X COORD", posPos2D.pinpointPos.getX(DistanceUnit.INCH));
            telemetry.addData("PINPOINT Y COORD", posPos2D.pinpointPos.getY(DistanceUnit.INCH));
            telemetry.addData("PINPOINT H COORD", posPos2D.pinpointPos.getHeading(AngleUnit.DEGREES));

            telemetry.addData("LIMELIGHT X COORD", posPos2D.limelightPos.getX(DistanceUnit.INCH));
            telemetry.addData("LIMELIGHT Y COORD", posPos2D.limelightPos.getY(DistanceUnit.INCH));
            telemetry.addData("LIMELIGHT H COORD", posPos2D.limelightPos.getHeading(AngleUnit.DEGREES));

            telemetry.addData("OTOS X VEL", velPos2D.otosPos.getX(DistanceUnit.INCH));
            telemetry.addData("OTOS Y VEL", velPos2D.otosPos.getY(DistanceUnit.INCH));
            telemetry.addData("OTOS H VEL", velPos2D.otosPos.getHeading(AngleUnit.DEGREES));

            telemetry.addData("PINPOINT X VEL", velPos2D.pinpointPos.getX(DistanceUnit.INCH));
            telemetry.addData("PINPOINT Y VEL", velPos2D.pinpointPos.getY(DistanceUnit.INCH));
            telemetry.addData("PINPOINT H VEL", velPos2D.pinpointPos.getHeading(AngleUnit.DEGREES));

            telemetry.addData("WEIGHTED X VEL", odometryFusion.getLinearXVelocity());
            telemetry.addData("WEIGHTED Y VEL", odometryFusion.getLinearYVelocity());
            telemetry.addData("WEIGHTED H VEL", odometryFusion.getAngularVelocity());

            telemetry.update();
        }

    }
}
