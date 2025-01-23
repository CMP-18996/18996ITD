package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
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

        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ScheduleCommand(
                        new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.READY)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new ScheduleCommand(
                        new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.DEPOSIT)
                )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        telemetry.addData("STATE", robot.deposit.getBucketState());
        telemetry.update();
    }
}
