package org.firstinspires.ftc.teamcode.common.robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HardwareMapNames {


    /*

    control hub ports

    0 -intake servo 1
    1 - intake servo 2
    2 - trapdoor
    3 - intake rotator
    4 - deposit rotator
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
    public static String BUCKET_SERVO = "bucket";
    public static String SPECIMEN_SERVO = "specimen";

    // Hang
    public static String HANG_MOTOR_1 = "hang";
    public static String HANG_MOTOR_2 = "hang1";

    // Odometry
    public static String OTOS = "otos";
    public static String PINPOINT = "pinpoint";
    public static String LIMELIGHT = "limelight";

    // Drive
    public static String LEFT_FRONT = "leftFront";
    public static String RIGHT_FRONT = "rightFront";
    public static String LEFT_BACK = "leftBack";
    public static String RIGHT_BACK = "rightBack";
}
