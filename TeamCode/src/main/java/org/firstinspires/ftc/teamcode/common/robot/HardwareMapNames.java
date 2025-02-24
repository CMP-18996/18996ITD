package org.firstinspires.ftc.teamcode.common.robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HardwareMapNames {


    /*

    Control Hub:

    ADD THESE!!!!

        Servo:
        0 - intake servo 1
        1 - intake rotator
        2 - trapdoor
        4 - bucket

        Motor:

        I2C:

        USB 3 (blue one):
        limelight

    Expansion Hub:

        Motors:
        0 - lift
        1 - extension
        2 - hang

     */


    // Intake
    public static String INTAKE_BOTTOM_PIVOT = "intakeBottomPivot";
    public static String INTAKE_COLOR_SENSOR = "colorSensor";
    public static String INTAKE_TOP_PIVOT = "intakeTopPivot";
    public static String INTAKE_ROLLER_1 = "intakeRoller1";
    public static String INTAKE_ROLLER_2 = "intakeRoller2";
    public static String INTAKE_PIVOT = "intakePivot";
    public static String INTAKE_CLAW = "intakeClaw";

    // Extension
    public static String EXTENSION_MOTOR = "extension";

    // Lift
    public static String LIFT_MOTOR = "lift";

    // Deposit
    public static String BUCKET_SERVO = "bucket";
    public static String DEPOSIT_TRAPDOOR = "depositTrapdoor";

    // Hang
    public static String HANG_MOTOR = "hang1";

    // Odometry
    public static String OTOS = "sensor_otos";
    public static String PINPOINT = "pinpoint";
    public static String LIMELIGHT = "limelight";

    // Drive
    public static String LEFT_FRONT = "leftFront";
    public static String RIGHT_FRONT = "rightFront";
    public static String LEFT_BACK = "leftBack";
    public static String RIGHT_BACK = "rightBack";

    // Specimen
    public static String ARM_MOTOR = "armServo";
    public static String WRIST_SERVO = "wristServo";
    public static String GRIPPER_SERVO = "armClaw";

    // Ultrasonics
    public static String RIGHT_SIDE_ULTRASONIC = "rightUltrasonic";
    public static String BACK_SIDE_ULTRASONIC = "backUltrasonic";
    public static String BACK_LEFT_SIDE_ULTRASONIC = "backLeftUltrasonic";
    public static String BACK_RIGHT_SIDE_ULTRASONIC = "backRightUltrasonic";
}
