package org.firstinspires.ftc.teamcode.common.odo;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class OdometryFusion {

    OdometryHardware odometryHardware;

    private static double otos_linear_weight = 1.0;
    private static double otos_angular_weight = 1.0;

    private static double pinpoint_linear_weight = 1.0;
    private static double pinpoint_angular_weight = 1.0;

    private static double limelight_linear_weight = 1.0;
    //private static double LIMELIGHT_ANGULAR_WEIGHT = 1.0; for now, this is useless because the limelight angular is the pinpoint angle

    private Vector2d linearVel;
    private double angularVel;

    public OdometryFusion(OdometryHardware odometryHardware) {
        this.odometryHardware = odometryHardware;
    }

    // should be static (i think?)
    public static void setLinearWeights(double otos, double pinpoint, double limelight) {
        otos_linear_weight = otos;
        pinpoint_linear_weight = pinpoint;
        limelight_linear_weight = limelight;
    }

    public static void setAngularWeights(double otos, double pinpoint) {
        otos_angular_weight = otos;
        pinpoint_angular_weight = pinpoint;
    }

    public void updateOdometryData() {
        OdometryHardware.MultiPose2D multiPose2D = odometryHardware.getPosData(true, true, true);

        double weightedXVel =  (multiPose2D.otosPos.getX(DistanceUnit.INCH) * otos_linear_weight +
                                multiPose2D.pinpointPos.getX(DistanceUnit.INCH) * pinpoint_linear_weight +
                                multiPose2D.limelightPos.getX(DistanceUnit.INCH) * limelight_linear_weight)
                                / (otos_linear_weight + pinpoint_linear_weight + limelight_linear_weight);

        double weightedYVel =  (multiPose2D.otosPos.getY(DistanceUnit.INCH) * otos_linear_weight +
                                multiPose2D.pinpointPos.getY(DistanceUnit.INCH) * pinpoint_linear_weight +
                                multiPose2D.limelightPos.getY(DistanceUnit.INCH) * limelight_linear_weight)
                                / (otos_linear_weight + pinpoint_linear_weight + limelight_linear_weight);

        double weightedAngularVel =  (multiPose2D.otosPos.getHeading(AngleUnit.DEGREES) * otos_angular_weight +
                                    multiPose2D.pinpointPos.getHeading(AngleUnit.DEGREES) * pinpoint_angular_weight)
                                    / (otos_angular_weight + pinpoint_angular_weight);

        linearVel = new Vector2d(weightedXVel, weightedYVel);
        angularVel = weightedAngularVel;
    }

    public PoseVelocity2d getPoseVelocity2d() {
        return new PoseVelocity2d(linearVel, angularVel);
    }

    public Vector2d getLinearVector() {
        return linearVel;
    }

    public double getAngularVelocity() {
        return angularVel;
    }

    public double getLinearXVelocity() {
        return linearVel.x;
    }

    public double getLinearYVelocity() {
        return linearVel.y;
    }
}
