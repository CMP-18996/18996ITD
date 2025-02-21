package org.firstinspires.ftc.teamcode.common.robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Drive {
    public static double X_MAX_RATE = 64.68235; // in/s
    public static double Y_MAX_RATE = 49.777325;
    public static double H_HAX_RATE = 5; //rad/s

    public static double X_CENTER_RATE = 10; // in/s
    public static double Y_CENTER_RATE = 10;
    public static double H_CENTER_RATE = 1; //rad/s

    public static double X_EXPO = 1; // in/s
    public static double Y_EXPO = 1;
    public static double H_EXPO = 1; //rad/s

    public static double kX = 0.01;
    public static double kY = 0.01;
    public static double kH = 0.01;

    public static double calculateXRate(double stickValue) {
        return (X_CENTER_RATE * stickValue) +
                (X_MAX_RATE - X_CENTER_RATE) *
                (Math.pow(stickValue, 6) + Math.pow(stickValue, 2) * (1 - X_EXPO));
    }

    public static double calculateYRate(double stickValue) {
        return (Y_CENTER_RATE * stickValue) +
                (Y_MAX_RATE - Y_CENTER_RATE) *
                        (Math.pow(stickValue, 6) + Math.pow(stickValue, 2) * (1 - Y_EXPO));
    }

    public static double calculateHRate(double stickValue) {
        return (H_CENTER_RATE * stickValue) +
                (H_HAX_RATE - H_CENTER_RATE) *
                        (Math.pow(stickValue, 6) + Math.pow(stickValue, 2) * (1 - H_EXPO));
    }

    public static double calculateXVectorComponent(double currentRate, double setRate) {
        double error = setRate - currentRate;
        return error * kX;
    }

    public static double calculateYVectorComponent(double currentRate, double setRate) {
        double error = setRate - currentRate;

        return error * kY;
    }
    public static double calculateHVectorComponent(double currentRate, double setRate) {
        double error = setRate - currentRate;
        return error * kH;
    }
}
