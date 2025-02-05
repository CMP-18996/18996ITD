package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetMotorState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeTrapdoorSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp(name = "Intake Test")
public class IntakeTest extends CommandOpMode {
    Subsystems subsystems = Subsystems.INTAKE;
    Robot robot;
    GamepadEx gamepad;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ScheduleCommand(
                        new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.TRANSFER),
                        new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.TRANSFER)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new ScheduleCommand(
                        new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.MOVING),
                        new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.MOVING)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ScheduleCommand(
                        new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.PICK_UP),
                        new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.PICK_UP)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new ScheduleCommand(
                        new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.REST),
                        new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.REST)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ScheduleCommand(
                        new IntakeSetMotorState_INST(robot.intake, IntakeSubsystem.IntakeMotorState.ACTIVE)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ScheduleCommand(
                        new IntakeSetMotorState_INST(robot.intake, IntakeSubsystem.IntakeMotorState.DISABLED)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new ScheduleCommand(
                        new IntakeSetMotorState_INST(robot.intake, IntakeSubsystem.IntakeMotorState.REVERSING)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new IntakeTrapdoorSetPosition_INST(robot.intake, IntakeSubsystem.IntakeTrapdoorState.OPEN)
                        ),
                        new ScheduleCommand(
                                new IntakeTrapdoorSetPosition_INST(robot.intake, IntakeSubsystem.IntakeTrapdoorState.CLOSED)
                        ),
                        () -> robot.intake.getTrapdoorState().equals(IntakeSubsystem.IntakeTrapdoorState.CLOSED)
                )
        );

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        telemetry.addData("ARM STATE", robot.intake.getIntakeArmState());
        telemetry.addData("WRIST STATE", robot.intake.getIntakeWristState());
        telemetry.addData("TRAPDOOR STATE", robot.intake.getTrapdoorState());
        telemetry.addData("ROLLER STATE", robot.intake.getIntakeRollerState());
        telemetry.addData("COLOR", robot.intake.getCurrentColor());
        telemetry.update();
    }
}
