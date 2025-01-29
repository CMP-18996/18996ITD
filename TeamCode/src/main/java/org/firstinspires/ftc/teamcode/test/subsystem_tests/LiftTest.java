package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp(name = "Lift Test")
public class LiftTest extends CommandOpMode {
    Subsystems subsystems = Subsystems.LIFT;
    Robot robot;
    GamepadEx gamepad;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new LiftSetPosition(robot.lift, LiftSubsystem.LiftState.HIGH_BUCKET)
                        ),
                        new ScheduleCommand(
                                new LiftSetPosition(robot.lift, LiftSubsystem.LiftState.TRANSFER)
                        ),
                        () -> robot.lift.getLiftState().equals(LiftSubsystem.LiftState.TRANSFER)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new LiftSetPosition(robot.lift, LiftSubsystem.LiftState.LOW_BUCKET)
                        ),
                        new ScheduleCommand(
                                new LiftSetPosition(robot.lift, LiftSubsystem.LiftState.TRANSFER)
                        ),
                        () -> robot.lift.getLiftState().equals(LiftSubsystem.LiftState.TRANSFER)
                )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        telemetry.addData("STATE", robot.lift.getLiftState());
        telemetry.addData("ERROR", robot.lift.getError());
        telemetry.update();
    }
}
