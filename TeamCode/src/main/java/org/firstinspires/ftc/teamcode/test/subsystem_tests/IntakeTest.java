package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetRollerState_INST;
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
        robot = new Robot(hardwareMap, Subsystems.INTAKE, Subsystems.DEPOSIT);
        gamepad = new GamepadEx(gamepad1);

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
                        new IntakeSetRollerState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.ACTIVE)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ScheduleCommand(
                        new IntakeSetRollerState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.DISABLED)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new ScheduleCommand(
                        new IntakeSetRollerState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.REVERSING)
                )
        );

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        telemetry.addData("ARM STATE", robot.intake.getIntakeArmState());
        telemetry.addData("WRIST STATE", robot.intake.getIntakeWristState());
        telemetry.addData("PIVOT STATE", robot.intake.getIntakePivotState());
        telemetry.addData("ROLLER STATE", robot.intake.getIntakeRollerState());
        telemetry.update();
    }
}
