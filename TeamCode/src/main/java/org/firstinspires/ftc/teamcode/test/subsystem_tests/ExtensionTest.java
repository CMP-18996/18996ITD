package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp(name = "Extension Test")
public class ExtensionTest extends CommandOpMode {
    Subsystems subsystems = Subsystems.EXTENSION;
    Robot robot;
    GamepadEx gamepad;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ScheduleCommand(
                        new ExtensionSetPosition(robot.extension, ExtensionSubsystem.ExtensionState.FULLY_EXTENDED)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ScheduleCommand(
                        new ExtensionSetPosition(robot.extension, ExtensionSubsystem.ExtensionState.TRANSFER)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new ScheduleCommand(
                        new ExtensionSetPosition(robot.extension, ExtensionSubsystem.ExtensionState.CUSTOM)
                )
        );

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        double power = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        if(power != 0) {
            robot.extension.setExtensionMotorPower(power);
        }

        telemetry.addData("STATE", robot.extension.getExtensionState());
        telemetry.addData("ERROR", robot.extension.getError());
        telemetry.update();
    }
}
