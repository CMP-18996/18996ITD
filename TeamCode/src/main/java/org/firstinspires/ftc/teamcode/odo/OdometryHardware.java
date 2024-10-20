package org.firstinspires.ftc.teamcode.odo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


/*
    The purpose of this class is or organize all the odo hardware
   OpMode will create instance of this class and pass it to OdometryFusion
   There are SO MANY different methods I could write for every possible action to variable number of sensors
   I have no idea what we will actually use so adding trivial methods may be needed if you don't see something

   TODO add limelight
   TODO add rev IMU (might as well even if its sucks, couldn't hurt as we don't have to use it)
   TODO bulk data reads

   TODO (maybe) rewrite the stupid sparkfun Pose2D system its really annoying
 */
@Config
public class OdometryHardware {
    private final SparkFunOTOS otos;
    private final GoBildaPinpointDriver pinpoint;

    private static double OTOS_LINEAR_SCALAR = 1.0;
    private static double OTOS_ANGULAR_SCALAR = 1.0;
    private static double OTOS_OFFSET_X = 0;
    private static double OTOS_OFFSET_Y = 0;
    private static double OTOS_OFFSET_H = 0;

    private static double PINPOINT_OFFSET_X = 0;
    private static double PINPOINT_OFFSET_Y = 0;

    private static double START_POS_X = 0;
    private static double START_POS_Y = 0;
    private static double START_POS_H = 0;

    public OdometryHardware(HardwareMap hardwareMap) {

        this.otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        configureOtos();

        configurePinpoint();

        otosCalibrateIMU();

        pinpointCalibrateIMU();

        setPosition(START_POS_X, START_POS_Y, START_POS_H);
    }

    public void setPosition(Pose2D pos) {
        otos.resetTracking(); // may not be necessary

        otos.setPosition(new SparkFunOTOS.Pose2D(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES))); // why did sparkfun make their own pose system

        // this doesn't work cuz it does imu as well, also probably not necessary
        //pinpoint.resetPosAndIMU();

        pinpoint.setPosition(pos);
    }

    public void setPosition(double x, double y, double h) {
        otos.resetTracking(); // may not be necessary

        otos.setPosition(new SparkFunOTOS.Pose2D(x, y, h));

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, h));
    }

    // ROBOT MUST BE STATIONARY
    public void otosCalibrateIMU() {
        otos.calibrateImu();
    }

    // ROBOT MUST BE STATIONARY
    public void pinpointCalibrateIMU() {
        pinpoint.recalibrateIMU();
    }

    // ROBOT MUST BE STATIONARY
    // good practice to setPosition() after if possible
    public void calibrateAll() {
        otosCalibrateIMU();

        pinpointCalibrateIMU();
    }

    private void configureOtos()
    {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(OTOS_OFFSET_X, OTOS_OFFSET_Y, OTOS_OFFSET_H);
        otos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        otos.setLinearScalar(OTOS_LINEAR_SCALAR);
        otos.setAngularScalar(OTOS_ANGULAR_SCALAR);
    }

    private void configurePinpoint()
    {
        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        pinpoint.setOffsets(PINPOINT_OFFSET_X, PINPOINT_OFFSET_Y); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    /* even goBilda says don't use this so... */
    public float getPinpointYawScalar() {
        return pinpoint.getYawScalar();
    }

    /* useless cant be bothered to actually write it */
    public void getVersion() {
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);

        pinpoint.getDeviceVersion();
    }
}
