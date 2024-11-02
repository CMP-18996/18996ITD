package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

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
    private DcMotorEx extension;
    private GamepadEx gamepad;

    @Override
    public void initialize() {
//        robot = new Robot(hardwareMap, Subsystems.INTAKE, Subsystems.DEPOSIT, Subsystems.LIFT);
        robot = new Robot(hardwareMap, Subsystems.INTAKE, Subsystems.DEPOSIT, Subsystems.LIFT);
        CommandScheduler.getInstance().reset();
        leftFront = hardwareMap.get(MotorEx.class, "leftFront");
        rightFront = hardwareMap.get(MotorEx.class, "rightFront");
        leftBack = hardwareMap.get(MotorEx.class, "leftBack");
        rightBack = hardwareMap.get(MotorEx.class, "rightBack");
        drive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);

        extension = hardwareMap.get(DcMotorEx.class, "extension");

        gamepad = new GamepadEx(gamepad1);
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
        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                () -> {
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
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                () -> {
                    CommandScheduler.getInstance().schedule(
                            new LiftSetPosition(robot.lift, LiftSubsystem.HIGH_BASKET)
                    );
                }
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                () -> {
                    CommandScheduler.getInstance().schedule(
                        new LiftSetPosition(robot.lift, LiftSubsystem.GROUND)
                );
            }
        );

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                () -> {
                    CommandScheduler.getInstance().schedule(
                        new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING)
                    );
                }
        );
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(
                () -> {
                    CommandScheduler.getInstance().schedule(
                        new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED)
                );
            }
        );

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                () -> {
                    CommandScheduler.getInstance().schedule(
                            new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.DROPPING),
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.ACTIVE)
                    );
                }
        );
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                () -> {
                    CommandScheduler.getInstance().schedule(
                            new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED)
                    );
                }
        );

//        CommandScheduler.getInstance().run();

        extension.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
    }
}
