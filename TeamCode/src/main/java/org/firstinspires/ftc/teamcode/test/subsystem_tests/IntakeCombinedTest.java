package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.ColorSensorCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.commands.TrapdoorCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp(name = "Intake Combined")
public class IntakeCombinedTest extends CommandOpMode {
    Subsystems subsystems = Subsystems.INTAKE;
    Robot robot;
    // NEED TO REVAMP
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        super.schedule(
                new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.ACTIVE),
                new WaitCommand(3000),
                new ColorSensorCommand(robot.intake, IntakeSubsystem.ColorState.BLUE)
        );
        if(robot.intake.colorState.toString().equals("Yellow") || robot.intake.colorState.toString().equals("Red")){
            super.schedule(
                new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.CLOSED),
                new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                new WaitCommand(3000),
                new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.CLOSED),
                new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.PICKING_UP)
            );
        }
        else if(robot.intake.colorState.toString().equals("Blue") || robot.intake.colorState.toString().equals("None")){
            super.schedule(
                new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.EJECTING),
                new WaitCommand(3000),
                new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.CLOSED)
            );
        }
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("Color:", robot.intake.colorState.toString());
        telemetry.update();
    }
}
