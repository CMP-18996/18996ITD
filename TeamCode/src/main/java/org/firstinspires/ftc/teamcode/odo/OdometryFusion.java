package org.firstinspires.ftc.teamcode.odo;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class OdometryFusion {

    OdometryHardware odometryHardware;

    private static double OTOS_LINEAR_WEIGHT = 1.0;
    private static double OTOS_ANGULAR_WEIGHT = 1.0;
    private static double PINPOINT_LINEAR_WEIGHT = 1.0;
    private static double PINPOINT_ANGULAR_WEIGHT = 1.0;

    public OdometryFusion(OdometryHardware odometryHardware) {
        this.odometryHardware = odometryHardware;
    }

    public void setLinearWeights(double otos, double pinpoint, double limelight) {

    }

    public void setAngularWeights(double otos, double pinpoint, double limelight, double rev) {

    }

    public static double getOtosLinearWeight() {
        return OTOS_LINEAR_WEIGHT;
    }

    public static void setOtosLinearWeight(double otosLinearWeight) {
        OTOS_LINEAR_WEIGHT = otosLinearWeight;
    }

    public static double getOtosAngularWeight() {
        return OTOS_ANGULAR_WEIGHT;
    }

    public static void setOtosAngularWeight(double otosAngularWeight) {
        OTOS_ANGULAR_WEIGHT = otosAngularWeight;
    }

    public static double getPinpointLinearWeight() {
        return PINPOINT_LINEAR_WEIGHT;
    }

    public static void setPinpointLinearWeight(double pinpointLinearWeight) {
        PINPOINT_LINEAR_WEIGHT = pinpointLinearWeight;
    }

    public static double getPinpointAngularWeight() {
        return PINPOINT_ANGULAR_WEIGHT;
    }

    public static void setPinpointAngularWeight(double pinpointAngularWeight) {
        PINPOINT_ANGULAR_WEIGHT = pinpointAngularWeight;
    }

    public void updateOdometryData() {

    }

    public PoseVelocity2d getPoseVelocity2d() {

    }

    public Vector2d getLinearVector() {

    }

    public double getAngularVelocity() {

    }

    public double getLinearXVelocity() {

    }

    public double getLinearYVelocity() {

    }
}
