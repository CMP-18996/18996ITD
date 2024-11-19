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
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.robot.Drive;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.common.odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.common.odo.OdometryHardware;

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
        gamepad = new GamepadEx(gamepad1);
        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                () -> {
                    super.schedule(
                            new SequentialCommandGroup(
                                    new LiftSetPosition(robot.lift, LiftSubsystem.HIGH_BASKET),
                                    new WaitCommand(400),
                                    new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.DEPOSITING),
                                    new WaitCommand(600),
                                    new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY),
                                    new LiftSetPosition(robot.lift, LiftSubsystem.LOW_BASKET)
                            )
                    );
                    telemetry.addLine("Y pressed, flip bucket");
                    telemetry.update();
                }
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                () -> {
                    super.schedule(
                            new LiftSetPosition(robot.lift, LiftSubsystem.HIGH_BASKET)
                    );
                    telemetry.addLine("Dpad up");
                    telemetry.addLine("" + robot.lift.motorWorking());
                    telemetry.update();
                }
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                () -> {
                    super.schedule(
                            new LiftSetPosition(robot.lift, LiftSubsystem.GROUND)
                    );
                    telemetry.addLine("Dpad down");
                    telemetry.addLine("" + robot.lift.motorWorking());

                    telemetry.update();
                }
        );

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                () -> {
                    super.schedule(
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING)
                    );
                    telemetry.addLine("Left bumper");
                    telemetry.update();
                }
        );
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(
                () -> {
                    super.schedule(
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED)
                    );
                }
        );

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                () -> {
                    CommandScheduler.getInstance().schedule(
                            //    new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.DROPPING),
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

//        robot = new Robot(hardwareMap, Subsystems.INTAKE, Subsystems.DEPOSIT, Subsystems.LIFT);
        robot = new Robot(hardwareMap, Subsystems.INTAKE, Subsystems.DEPOSIT, Subsystems.LIFT);
        CommandScheduler.getInstance().reset();
//        leftFront = hardwareMap.get(Motor.class, "lF");
//        rightFront = hardwareMap.get(Motor.class, "rF");
//        leftBack = hardwareMap.get(Motor.class, "lB");
//        rightBack = hardwareMap.get(Motor.class, "rB");
//        drive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);
        drive = new Drive(hardwareMap);
        odo = new OdometryHardware(hardwareMap);
        drive.setDriveMode(Drive.DriveMode.FIELD_CENTRIC);

        extension = hardwareMap.get(DcMotorEx.class, "extension");


        telemetry.addLine("Initialized");
        telemetry.update();
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
//        drive.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        odo.pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        drive.vectorDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, odo.pinpoint.getHeading());

        // deposit, flips bucket, retracts lift

        CommandScheduler.getInstance().run();

        extension.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        telemetry.addData("Lift target", robot.lift.getCurrTarget());
        telemetry.addData("Lift current position", robot.lift.getCurrentPosition());
        telemetry.update();
    }
}
