package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.commands.DepositRotationCommand;
import org.firstinspires.ftc.teamcode.common.commands.HangCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.RetractAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.robot.Drive;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.common.robot.OdometryHardware;

@TeleOp(name="Very Basic Teleop")
public class VeryBasicTeleop extends CommandOpMode {
    private Robot robot;
    private Drive drive;
//    private Motor leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx extension;
    private GamepadEx gamepad;
    OdometryHardware odo;

    @Override
    public void initialize() {
//        robot = new Robot(hardwareMap, Subsystems.INTAKE, Subsystems.DEPOSIT, Subsystems.LIFT);
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, Subsystems.ALL);
//        leftFront = hardwareMap.get(Motor.class, "lF");
//        rightFront = hardwareMap.get(Motor.class, "rF");
//        leftBack = hardwareMap.get(Motor.class, "lB");
//        rightBack = hardwareMap.get(Motor.class, "rB");
//        drive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);
        drive = new Drive(hardwareMap);
        //odo = new OdometryHardware(hardwareMap);

        extension = hardwareMap.get(DcMotorEx.class, "extension");

        gamepad = new GamepadEx(gamepad1);
        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                () -> {
                    schedule(
                            new SequentialCommandGroup(
                                    new LiftSetPosition(robot.lift, LiftSubsystem.HIGH_BASKET),
                                    new WaitCommand(2000),
                                    new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.DEPOSITING),
                                    new WaitCommand(1000),
                                    new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY),
                                    new LiftSetPosition(robot.lift, LiftSubsystem.LOW_BASKET)
                            )
                    );
                    telemetry.addLine("Y pressed, flip bucket");
                    telemetry.update();
                }
        );

        gamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                () -> {
                    schedule(
                            new HangCommand(robot.hang, HangSubsystem.HangPosition.L2)
                    );
                }
        );

        gamepad.getGamepadButton(GamepadKeys.Button.X).whenReleased(
                () -> {
                    schedule(
                            new HangCommand(robot.hang, HangSubsystem.HangPosition.L2_HUNG)
                    );
                    telemetry.addLine("should be hanged at low rung");
                    telemetry.update();
                }
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                () -> {
                    schedule(
                            new LiftSetPosition(robot.lift, LiftSubsystem.HIGH_BASKET)
                    );
                    telemetry.addLine("Dpad up");
                    telemetry.addLine("" + robot.lift.motorWorking());
                    telemetry.update();
                }
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                () -> {
                    schedule(
                            new LiftSetPosition(robot.lift, LiftSubsystem.GROUND)
                    );
                    telemetry.addLine("Dpad down");
                    telemetry.addLine("" + robot.lift.motorWorking());

                    telemetry.update();
                }
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                () -> {
                    schedule(
                            new RetractAndTransferCommand(robot.extension, robot.intake, robot.deposit)
                    );
                    telemetry.addLine("Dpad left");
                    telemetry.update();
                }
        );

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                () -> {
                    schedule(
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING)
                    );
                    telemetry.addLine("Left bumper");
                    telemetry.update();
                }
        );
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(
                () -> {
                    schedule(
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED)
                    );
                }
        );

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                () -> {
                    schedule(
                            //    new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.DROPPING),
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.ACTIVE)
                    );
                }
        );
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                () -> {
                    schedule(
                            new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED)
                    );
                }
        );


        telemetry.addLine("Initialized");
        telemetry.update();
    }

    /*
    Triangle - deposit, flips bucket, retracts lift
    Circle - nothing
    Cross - nothing
    Square - nothing
    Dpad up - lift to highest
    Dpad left - nothing lift to low bar
    Dpad right - nothing lift to high bar
    Dpad down - lift to lowest
    dpad press the same thing goes to 0
    Left trigger - extension back, variable
    Right trigger - extension out, variable
    Left button - held down, intake reverses
    Right button - toggle intake rotator up to down, turns intake on or off
     */
    public void run() {
//        drive.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        //odo.pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        //drive.vectorDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, odo.pinpoint.getHeading());

        // deposit, flips bucket, retracts lift
        CommandScheduler.getInstance().run();
        
        telemetry.addData("Lift target", robot.lift.getCurrTarget());
        telemetry.addData("Lift current position", robot.lift.getCurrentPosition());
        telemetry.update();
    }
}
