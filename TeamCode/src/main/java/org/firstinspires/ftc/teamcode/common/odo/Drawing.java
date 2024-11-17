package org.firstinspires.ftc.teamcode.common.odo;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * Ftc dashboard drawing class
 */
public class Drawing
{
    private static double ROBOT_SIZE = 18;

    // TODO add front identifier??
    public static void DrawRobot(Canvas canvas, Pose2D pos, String color) {
        canvas.setStrokeWidth(1)
                .setStroke(color)
                //.setFill("red")
                .setAlpha(1.0);

        // Create a box with robot size
        double halfRobotSize = ROBOT_SIZE/2;
        double[] bxPoints = {halfRobotSize, -halfRobotSize, -halfRobotSize, halfRobotSize};
        double[] byPoints = {halfRobotSize, halfRobotSize, -halfRobotSize, -halfRobotSize};

        // Rotate box to robot orientation
        rotatePoints(bxPoints, byPoints, pos.getHeading(AngleUnit.RADIANS));

        // Move box to robot position
        for (int i = 0; i < 4; i++) {
            bxPoints[i] += pos.getX(DistanceUnit.INCH);
            byPoints[i] += pos.getY(DistanceUnit.INCH);
        }

        // Draw box
        canvas.strokePolygon(bxPoints, byPoints);
    }

    public static void DrawVector(Canvas canvas, Pose2D pos, double magnitude, String color) {
        canvas.setStrokeWidth(1)
                .setStroke(color)
                //.setFill("red")
                .setAlpha(1.0);

        // Create an arrow with magnitude
        double[] bxPoints = {0, 0};
        double[] byPoints = {0, magnitude};

        // Rotate arrow to vector direction
        rotatePoints(bxPoints, byPoints, pos.getHeading(AngleUnit.RADIANS));

        // Move arrow to vector position
        for (int i = 0; i < 4; i++) {
            bxPoints[i] += pos.getX(DistanceUnit.INCH);
            byPoints[i] += pos.getY(DistanceUnit.INCH);
        }

        canvas.strokePolyline(bxPoints, byPoints);
    }

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }

    public static void setRobotSize(double robotSize) {
        ROBOT_SIZE = robotSize;
    }
}
