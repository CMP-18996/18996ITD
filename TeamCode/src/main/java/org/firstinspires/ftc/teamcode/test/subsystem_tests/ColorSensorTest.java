package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.commands.ColorSensorCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;


@TeleOp(name = "Color Sensor")
public class ColorSensorTest extends CommandOpMode {

    Subsystems subsystems = Subsystems.INTAKE;
    Robot robot;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        super.schedule(
            new SequentialCommandGroup(
                new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.ACTIVE),
                new WaitCommand(3000),
                new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED),
                new ColorSensorCommand(robot.intake, IntakeSubsystem.ColorState.YELLOW)
            )
        );
    }

    @Override
    public void run() {
    CommandScheduler.getInstance().run();
    }
}
