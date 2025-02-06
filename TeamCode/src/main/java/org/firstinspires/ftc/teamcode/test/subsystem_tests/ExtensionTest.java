package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.extension.ZeroExtension;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.lift.ZeroLift;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp(name = "Extension Test")
public class ExtensionTest extends CommandOpMode {
    Subsystems subsystems = Subsystems.EXTENSION;
    Robot robot;
    GamepadEx gamepad;

    private double previousPower = 0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new ZeroExtension(robot.extension),
                                new ExtensionSetPosition(robot.extension, ExtensionSubsystem.ExtensionState.EXTENDED)
                        ),
                        new ScheduleCommand(
                                new ExtensionSetPosition(robot.extension, ExtensionSubsystem.ExtensionState.TRANSFER)
                        ),
                        () -> robot.extension.getExtensionState().equals(ExtensionSubsystem.ExtensionState.TRANSFER)
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

        if(power != previousPower) {
            robot.extension.setExtensionMotorPower(power);
            previousPower = power;
        }

        telemetry.addData("STATE", robot.extension.getExtensionState());
        telemetry.addData("ERROR", robot.extension.getError());
        telemetry.update();
    }
}
