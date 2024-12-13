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
    public static String INTAKE_SERVO_1 = "intake1";
    public static String INTAKE_SERVO_2 = "intake2";
    public static String INTAKE_ROTATOR = "intakeRotator";
    public static String INTAKE_TRAPDOOR = "trapdoor";
    public static String INTAKE_COLOR_SENSOR = "colorSensor";

    // Extension
    public static String EXTENSION_MOTOR = "extension";

    // Lift
    public static String LIFT_MOTOR = "lift";

    // Deposit
    public static String BUCKET_SERVO = "bucket";

    // Specimen
    public static String SPECIMEN_SERVO = "specimen";

    // Hang
    public static String HANG_MOTOR_1 = "hang1";
    public static String HANG_MOTOR_2 = "hang2";

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
    public static String ARM_MOTOR = "armMotor";
    public static String WRIST_SERVO = "armServo";
    public static String GRIPPER_SERVO = "armClaw";
}
