package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.commands.DepositRotationCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
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
                    new SequentialCommandGroup(
                            new LiftSetPosition(robot.lift, LiftSubsystem.HIGH_BASKET),
                            new WaitCommand(400),
                            new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.DEPOSITING),
                            new WaitCommand(600),
                            new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY),
                            new LiftSetPosition(robot.lift, LiftSubsystem.LOW_BASKET)
                    )
            );
        }

        if (gamepad1.dpad_up) {
            CommandScheduler.getInstance().schedule(
                    new LiftSetPosition(robot.lift, LiftSubsystem.HIGH_BASKET)
            );
        }

        if (gamepad1.dpad_down) {
            CommandScheduler.getInstance().schedule(
                    new LiftSetPosition(robot.lift, LiftSubsystem.GROUND)
            );
        }

        if (gamepad1.left_bumper) {
            CommandScheduler.getInstance().schedule(
                    new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING)
            );
        }
        else {
            CommandScheduler.getInstance().schedule(
                    new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED)
            );
        }

        if (gamepad1.right_bumper) {
            CommandScheduler.getInstance().schedule(
                    new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.DROPPING),
                    new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.ACTIVE)
            );
        }
        else {
            CommandScheduler.getInstance().schedule(
                    new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                    new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED)
            );
        }

        CommandScheduler.getInstance().run();
    }
}
