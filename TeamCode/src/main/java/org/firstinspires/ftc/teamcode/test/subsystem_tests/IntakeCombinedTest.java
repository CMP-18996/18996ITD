package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.commands.ColorSensorCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.commands.TrapdoorCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp(name = "Intake Combined")
public class IntakeCombinedTest extends CommandOpMode {
    IntakeSubsystem intakeSubsystem;
    Subsystems subsystems = Subsystems.INTAKE;
    Robot robot;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        super.schedule(
                new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.ACTIVE),
                new WaitCommand(3000),
                new ColorSensorCommand(intakeSubsystem)
        );
        if(intakeSubsystem.colorState.toString().equals("Yellow") || intakeSubsystem.colorState.toString().equals("Red")){
            super.schedule(
                new TrapdoorCommand(intakeSubsystem, IntakeSubsystem.TrapdoorState.TRANSFERRING),
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                new WaitCommand(3000),
                new TrapdoorCommand(intakeSubsystem, IntakeSubsystem.TrapdoorState.CLOSED),
                new IntakeRotatorCommand(intakeSubsystem, IntakeSubsystem.IntakeRotatorState.DROPPING)
            );
        }
        else if(intakeSubsystem.colorState.toString().equals("Blue") || intakeSubsystem.colorState.toString().equals("None")){
            super.schedule(
                new TrapdoorCommand(intakeSubsystem, IntakeSubsystem.TrapdoorState.EJECTING),
                new WaitCommand(3000),
                new TrapdoorCommand(intakeSubsystem, IntakeSubsystem.TrapdoorState.CLOSED)
            );
        }
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("Color:", intakeSubsystem.colorState.toString());
        telemetry.update();
    }
}
