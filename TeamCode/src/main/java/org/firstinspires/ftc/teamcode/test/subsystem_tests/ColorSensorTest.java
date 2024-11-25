package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.TrapdoorCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@Config
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
            ),
            new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.CLOSED)
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("red:", colorSensor.red());
        telemetry.addData("blue:", colorSensor.blue());
        telemetry.addData("green:", colorSensor.green());

        // This conditional stays until the color stuff gets tuned
        if (colorSensor.blue() > 150 || colorSensor.green() > 150) {
//            occupied = true;
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            // new WaitCommand(300),
                            new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.CLOSED),
                            new WaitCommand(400),
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED)
                    )
            );
        }
        if (colorSensor.blue() <= 150 && colorSensor.green() <= 150) {
//            occupied = false;
            CommandScheduler.getInstance().schedule(
                    new WaitCommand(1000),
                    new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.EJECTING),
                    new WaitCommand(200),
                    new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.ACTIVE)
            );
        }
    }
}
