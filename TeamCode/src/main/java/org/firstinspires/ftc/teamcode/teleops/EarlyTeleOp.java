package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.commands.DepositRotationCommand;
import org.firstinspires.ftc.teamcode.common.commands.ExtendCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.TrapdoorCommand;
import org.firstinspires.ftc.teamcode.common.robot.Drive;
import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.Team;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp
public class EarlyTeleOp extends CommandOpMode {
    private Team team = Team.RED;
    private Team oppositeTeam;

    private Robot robot;
    private GamepadEx gamepad;
    private boolean alreadyTransferred = false;
    private Drive drive;

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap);

        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap, Subsystems.ALL);
        gamepad = new GamepadEx(gamepad1);

        if (team.equals(Team.RED)) oppositeTeam = Team.BLUE;
        else oppositeTeam = Team.RED;

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new LiftSetPosition(robot.lift, robot.lift.HIGH_BASKET),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.READY_TO_DEPOSIT)
                        ),
                        new ScheduleCommand(
                                new LiftSetPosition(robot.lift, robot.lift.GROUND),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY)
                        ),
                        () -> robot.lift.getCurrTarget() != robot.lift.HIGH_BASKET
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new LiftSetPosition(robot.lift, robot.lift.LOW_BASKET),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.READY_TO_DEPOSIT)
                        ),
                        new ScheduleCommand(
                                new LiftSetPosition(robot.lift, robot.lift.GROUND),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY)
                        ),
                        () -> robot.lift.getCurrTarget() != robot.lift.LOW_BASKET
                )
        );

        // A is Cross
        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.DEPOSITING),
                                new WaitCommand(1400),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY),
                                new LiftSetPosition(robot.lift, robot.lift.GROUND)
                        ),
                        new SequentialCommandGroup(
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.DEPOSITING),
                                new WaitCommand(1400),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY)
                        ),
                        () -> robot.lift.getCurrentPosition() != robot.lift.GROUND
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.ACTIVE),
                                new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.CLOSED),
                                new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.PICKING_UP)
                        ),
                        new ScheduleCommand(
                                new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED),
                                new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.MOVING)
                        ),
                        () -> robot.intake.getIntakingState().equals(IntakeSubsystem.IntakingState.DISABLED)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING)
        ).whenReleased(
                new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED)
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        if (robot.extension.getState().equals(ExtensionSubsystem.ExtensionState.CUSTOM)){
            double rawExtensionPower = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            robot.extension.setPower(Math.pow(rawExtensionPower, 3));
        }


        Team detectedColor = robot.intake.updateColorState2();
        /*
        if (detectedColor.equals(oppositeTeam)) {
            schedule(
                    new SequentialCommandGroup(
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED),
                            new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.EJECTING),
                            new WaitCommand(200),
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.ACTIVE)
                    )
            );
        }

        else if (detectedColor.equals(team) && !alreadyTransferred) {
            alreadyTransferred = true;

            schedule(
                    new SequentialCommandGroup(
                            new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.MOVING),
                            new WaitCommand(300),
                            new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                            new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.CONTRACTED),
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING),
                            new InstantCommand(() -> {
                                alreadyTransferred = false;
                                robot.extension.setState(ExtensionSubsystem.ExtensionState.CUSTOM);
                            })
                    )
            );
        } */

        drive.robotCentricDrive(gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x);

        telemetry.addData("Detected Color:", detectedColor);
        telemetry.addData("Extension State:", robot.extension.getState().toString());
        telemetry.update();
    }
}
