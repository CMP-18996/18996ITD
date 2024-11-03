package org.firstinspires.ftc.teamcode.odo;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class LinearPathing {
    private Drive drive;
    private OdometryFusion odometryFusion;

    private static double kX = 0.1;
    private static double kY = 0.1;
    private static double kH = 0.1;


    public LinearPathing(Drive drive, OdometryFusion odometryFusion) {
        this.drive = drive;
        this.odometryFusion = odometryFusion;
    }

    public void pointToPointDrive(Pose2D goalPos, Pose2D currentPos) {

        //Pose2D currentPos = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

        double xPID = xPID(currentPos.getX(DistanceUnit.INCH));
        double yPID = yPID(currentPos.getY(DistanceUnit.INCH));
        double hPID = hPID(currentPos.getHeading(AngleUnit.DEGREES));

        double rad = currentPos.getHeading(AngleUnit.RADIANS);
        double x_power = (xPID * Math.cos(rad)) - (yPID * Math.sin(rad));
        double y_power = (yPID * Math.cos(rad)) - (xPID * Math.sin(rad));
        double h_power = hPID;

        drive.vectorDrive(x_power, y_power, h_power);
    }

    private double xPID(double x) {
        return kX * x;
    }

    private double yPID(double y) {
        return kY * y;
    }

    private double hPID(double h) {
        return kH * h;
    }
}
