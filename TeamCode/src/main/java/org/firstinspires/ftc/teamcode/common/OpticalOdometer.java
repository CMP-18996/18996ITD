package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OpticalOdometer {
    SparkFunOTOS optOdo;
    public OpticalOdometer(HardwareMap hardwareMap) {
        optOdo = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
    }
    public OpticalOdometer(SparkFunOTOS optOdo) {
        this.optOdo = optOdo;
    }
    public void configure(Pose2d startPose) {
        optOdo.setLinearUnit(DistanceUnit.INCH);
        optOdo.setAngularUnit(AngleUnit.DEGREES);

        //offset on the robot
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0,0,0);
        optOdo.setOffset(offset);

        optOdo.setLinearScalar(1.0);
        optOdo.setAngularScalar(1.0);

        optOdo.calibrateImu();
        optOdo.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(startPose.position.x, startPose.position.y, Math.toDegrees(startPose.heading.toDouble()));
        optOdo.setPosition(currentPosition);
    }
    public SparkFunOTOS getOtos() {
        return optOdo;
    }
    public void setOtos(SparkFunOTOS optOdo) {
        this.optOdo = optOdo;
    }
}
