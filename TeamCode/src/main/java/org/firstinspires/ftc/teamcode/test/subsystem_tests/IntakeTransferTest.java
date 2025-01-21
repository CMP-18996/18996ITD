package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.DepositRotationCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeArmPivotCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
@TeleOp(name = "Intake Test")
public class IntakeTransferTest extends CommandOpMode {
    Subsystems intake = Subsystems.INTAKE;
    Subsystems deposit = Subsystems.DEPOSIT;
    Robot robot;
    private GamepadEx gamepad_1;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, intake, deposit);
        gamepad_1 = new GamepadEx(gamepad1);

        gamepad_1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.MOVING),
                                new IntakeArmPivotCommand(robot.intake, IntakeSubsystem.IntakeArmPivotState.MOVING)
                        ),
                        new ScheduleCommand(
                                new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                                new IntakeArmPivotCommand(robot.intake, IntakeSubsystem.IntakeArmPivotState.TRANSFERRING)
                        ),
                        () -> robot.intake.getIntakeRotatorState().equals(IntakeSubsystem.IntakeRotatorState.TRANSFERRING)
                )
        );
    }

    @Override
    public void run() {
        telemetry.addData("Intake Rotator State", robot.intake.getIntakeRotatorState());
        telemetry.addData("VA", 0);
        telemetry.update();

        CommandScheduler.getInstance().run();
    }
}
