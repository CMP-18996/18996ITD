package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositTrapdoorPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp(name = "Deposit Test")
public class DepositTest extends CommandOpMode {
    Subsystems subsystems = Subsystems.DEPOSIT;
    Robot robot;
    GamepadEx gamepad;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ScheduleCommand(
                        new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.TRANSFER)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new ScheduleCommand(
                        new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.DEPOSIT)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ScheduleCommand(
                        new DepositTrapdoorPosition_INST(robot.deposit, DepositSubsystem.DepositTrapdoorState.OPEN)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new ScheduleCommand(
                        new DepositTrapdoorPosition_INST(robot.deposit, DepositSubsystem.DepositTrapdoorState.CLOSED)
                )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        telemetry.addData("STATE", robot.deposit.getBucketState());
        telemetry.addData("TRAPDOOR", robot.deposit.getDepositTrapdoorState());
        telemetry.update();
    }
}
