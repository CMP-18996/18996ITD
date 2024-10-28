package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.commands.ColorSensorCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.TrapdoorCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;


@TeleOp(name = "Color Sensor")
public class ColorSensorTest extends CommandOpMode {

    Subsystems subsystems = Subsystems.INTAKE;
    Robot robot;
    ColorSensor colorSensor;
//    boolean occupied;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
//        occupied = false;
        robot = new Robot(hardwareMap, subsystems);
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        super.schedule(
            new SequentialCommandGroup(
                new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.ACTIVE),
                new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.CLOSED)
                // new WaitCommand(3000),
                // new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED),
                // new ColorSensorCommand(robot.intake, IntakeSubsystem.ColorState.YELLOW)
            )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        // This conditional stays until the color stuff gets tuned
        if (colorSensor.blue() > 150 || colorSensor.green() > 150) {
//            occupied = true;
            CommandScheduler.getInstance().schedule(
                    new ParallelCommandGroup(
                            new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.CLOSED),
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED)
                    )
            );
        }
        if (colorSensor.blue() <= 150 && colorSensor.green() <= 150) {
//            occupied = false;
            CommandScheduler.getInstance().schedule(
                    new WaitCommand(500),
                    new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.EJECTING),
                    new WaitCommand(200),
                    new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.ACTIVE)
            );
        }
    }
}
