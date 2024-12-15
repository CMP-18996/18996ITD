package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorManualCommand;

@Disabled
@TeleOp(name = "Intake manual tuner")
public class IntakeManualTunerTest extends CommandOpMode {
    Subsystems subsystems = Subsystems.INTAKE;
    Robot robot;
    GamepadEx gamepad;
    double pos = 0.42;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, Subsystems.INTAKE);
        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> pos += 0.01),
                    new IntakeRotatorManualCommand(robot.intake, pos)
                )
        );
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> pos -= 0.01),
                        new IntakeRotatorManualCommand(robot.intake, pos)
                )
        );
        super.schedule(new IntakeRotatorManualCommand(robot.intake, pos));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("position", pos);
        telemetry.update();
    }
}
