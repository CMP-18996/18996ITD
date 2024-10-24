package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.commands.TrapdoorCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp(name="Trapdoor Test")
public class TrapdoorTest extends CommandOpMode {
    Subsystems subsystems = Subsystems.INTAKE;
    Robot robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        super.schedule(
                new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.EJECTING),
                new WaitCommand(3000),
                new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.TRANSFERRING),
                new WaitCommand(3000),
                new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.CLOSED)
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
