package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

@Config
@TeleOp(name = "RTIntakeTest")
public class RTIntakeTest extends LinearOpMode {

    public static double ARM_PICK_UP_POS = 0.5;
    public static double ARM_MOVING_POS = 0.4;
    public static double ARM_REST_POS = 0.2;

    public static double WRIST_PICK_UP_POS = 0.5;
    public static double WRIST_MOVING_POS = 0.4;
    public static double WRIST_REST_POS = 0.8;

    public static double PIVOT_0_DEG = 0;
    public static double PIVOT_90_DEG = 0.5;

    private boolean pivot_rotated = false;

    private CRServo intakeRollers;
    private Servo intakePivot;
    private Servo intakeWrist;
    private Servo intakeArm;

    @Override
    public void runOpMode() {
        intakeRollers = hardwareMap.get(CRServo.class, "intakeRollers");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        intakeWrist = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_TOP_PIVOT);
        intakeArm = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_BOTTOM_PIVOT);

        //intakeRollers.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeArm.setPosition(ARM_REST_POS);
        intakeWrist.setPosition(WRIST_REST_POS);
        intakePivot.setPosition(PIVOT_0_DEG);

        waitForStart();

        while(opModeIsActive()) {

            if(gamepad1.square) {
                pivot_rotated = !pivot_rotated;

                if(pivot_rotated) {
                    intakePivot.setPosition(PIVOT_90_DEG);
                }
                else {
                    intakePivot.setPosition(PIVOT_0_DEG);
                }
            }

            if(gamepad1.cross) {
                intakeArm.setPosition(ARM_PICK_UP_POS);
                intakeWrist.setPosition(WRIST_PICK_UP_POS);
                intakeRollers.setPower(1.0);
            }
            else if(gamepad1.circle) {
                intakeArm.setPosition(ARM_MOVING_POS);
                intakeWrist.setPosition(WRIST_MOVING_POS);
                intakeRollers.setPower(0.0);
            }
            else if(gamepad1.triangle) {
                intakeArm.setPosition(ARM_REST_POS);
                intakeWrist.setPosition(WRIST_REST_POS);
                intakeRollers.setPower(0.0);
                pivot_rotated = false;
                intakePivot.setPosition(PIVOT_0_DEG);
            }
        }
    }
}

