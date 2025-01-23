package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetArmPosition;
import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetGripperPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp(name =  "Specimen Stuff v1")
@Config
public class SpecimenTest extends CommandOpMode {
    Subsystems subsystems = Subsystems.SPECIMEN;
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
                                new SpecimenSetArmPosition(robot.specimen, SpecimenSubsystem.SpecimenArmState.CHAMBER)
                        ),
                        new ScheduleCommand(
                                new SpecimenSetArmPosition(robot.specimen, SpecimenSubsystem.SpecimenArmState.WALL)
                        ),
                        () -> robot.specimen.getSpecimenArmState() != SpecimenSubsystem.SpecimenArmState.CHAMBER
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.CLOSED)
                        ),
                        new ScheduleCommand(
                                new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.OPEN)
                        ),
                        () -> robot.specimen.getSpecimenGripperState() != SpecimenSubsystem.SpecimenGripperState.CLOSED
                )
        );

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        if (gamepad1.left_stick_y != 0) { robot.specimen.manualAdjustArm((int) (-gamepad1.left_stick_y * 5)); }
        if (gamepad1.right_stick_y != 0) { robot.specimen.manualAdjustWrist(-gamepad1.right_stick_y/30); }
    }
}
