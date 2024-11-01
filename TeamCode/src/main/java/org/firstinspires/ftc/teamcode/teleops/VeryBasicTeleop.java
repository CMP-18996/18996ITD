package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp(name="Very Basic Teleop")
public class VeryBasicTeleop extends CommandOpMode {
    private Robot robot;
    private MecanumDrive drive;
    private MotorEx leftFront, rightFront, leftBack, rightBack;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, Subsystems.INTAKE, Subsystems.DEPOSIT, Subsystems.EXTENSION, Subsystems.LIFT);
        CommandScheduler.getInstance().reset();
        leftFront = hardwareMap.get(MotorEx.class, "leftFront");
        rightFront = hardwareMap.get(MotorEx.class, "rightFront");
        leftBack = hardwareMap.get(MotorEx.class, "leftBack");
        rightBack = hardwareMap.get(MotorEx.class, "rightBack");
        drive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);
    }

    /*
    Triangle - deposit, flips bucket, retracts lift
    Circle - nothing
    Cross - nothing
    Square nothing
    Dpad up - lift to highest
    Dpad left - nothing
    Dpad right - nothing
    Dpad down - lift to lowest
    Left trigger - extension back, variable
    Right trigger - extension out, variable
    Left button - held down, intake reverses
    Right button - toggle intake rotator up to down, turns intake on or off
     */
    public void run() {
        drive.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        // deposit, flips bucket, retracts lift
        if (gamepad1.triangle) {
            CommandScheduler.getInstance().schedule(
//                    new
            );
        }

        if (gamepad1.dpad_up) {

        }
    }
}
