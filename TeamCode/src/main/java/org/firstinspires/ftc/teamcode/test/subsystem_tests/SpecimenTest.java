package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
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
import org.firstinspires.ftc.teamcode.common.commands.HangCommand;
import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.SpecimenArmCommand;
import org.firstinspires.ftc.teamcode.common.commands.SpecimenGripperCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.teleops.SPECIKEMARMTET;

@Disabled
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
                                new SpecimenArmCommand(robot.specimen, SpecimenSubsystem.SpecimenPosition.CHAMBER)
                        ),
                        new ScheduleCommand(
                                new SpecimenArmCommand(robot.specimen, SpecimenSubsystem.SpecimenPosition.WALL)
                        ),
                        () -> robot.specimen.getSpecimenState() != SpecimenSubsystem.SpecimenPosition.CHAMBER
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.CLOSED)
                        ),
                        new ScheduleCommand(
                                new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.OPEN)
                        ),
                        () -> robot.specimen.getGripperPosition() != SpecimenSubsystem.GripperPosition.CLOSED
                )
        );

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        if (gamepad1.left_stick_y != 0) { robot.specimen.manualAdjustArm((int) (-gamepad1.left_stick_y * 5)); }
        if (gamepad1.right_stick_y != 0) { robot.specimen.manualAdjustWrist(-gamepad1.right_stick_y/30); }

        telemetry.addData("POWER", robot.specimen.power);
        telemetry.addData("ERR", robot.specimen.error);
        telemetry.addData("POSITION", robot.specimen.getSpecimenState());
        telemetry.addData("GRI", robot.specimen.getGripperPosition());
        telemetry.addData("POS", robot.specimen.armMotor.getCurrentPosition());
        telemetry.addData("SETPOINT", robot.specimen.armTarget);
        telemetry.addData("WRIST", robot.specimen.wristTarget);

        telemetry.update();
    }
}
