package org.firstinspires.ftc.teamcode.common.robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Drive {
    public static double X_MAX_RATE = 65; // in/s
    public static double Y_MAX_RATE = 50;
    public static double H_HAX_RATE = 5; //rad/s

    public static double X_CENTER_RATE = 65; // in/s
    public static double Y_CENTER_RATE = 50;
    public static double H_CENTER_RATE = 5; //rad/s

    public static double X_EXPO = 0.0; // in/s
    public static double Y_EXPO = 0.0;
    public static double H_EXPO = 0.0; //rad/s

    public static double kX = 0.01;
    public static double kY = 0.01;
    public static double kH = 0.01;

    public static double calculateXRate(double stickValue) {
        double x = Math.abs(stickValue);
        return Math.signum(stickValue) *
                (X_CENTER_RATE * x) +
                (X_MAX_RATE - X_CENTER_RATE) *
                (Math.pow(x, 6) + Math.pow(x, 2) * (1 - X_EXPO));
    }

    public static double calculateYRate(double stickValue) {
        double x = Math.abs(stickValue);
        return Math.signum(stickValue) *
                (Y_CENTER_RATE * x) +
                (Y_MAX_RATE - Y_CENTER_RATE) *
                (Math.pow(x, 6) + Math.pow(x, 2) * (1 - Y_EXPO));
    }

    public static double calculateHRate(double stickValue) {
        double x = Math.abs(stickValue);
        return Math.signum(stickValue) *
                (H_CENTER_RATE * x) +
                (H_HAX_RATE - H_CENTER_RATE) *
                (Math.pow(x, 6) + Math.pow(x, 2) * (1 - H_EXPO));
    }

    public static double calculateXVectorDelta(double currentRate, double setRate) {
        double error = setRate - currentRate;
        return error * kX;
    }

    public static double calculateYVectorDelta(double currentRate, double setRate) {
        double error = setRate - currentRate;
        return error * kY;
    }
    public static double calculateHVectorDelta(double currentRate, double setRate) {
        double error = setRate - currentRate;
        return error * kH;
    }
}
