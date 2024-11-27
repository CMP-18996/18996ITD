package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.HangCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp(name="Hang Test")
public class HangCommandTest extends CommandOpMode {
    Subsystems subsystems = Subsystems.HANG;
    Robot robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        super.schedule(
                new HangCommand(robot.hang, HangSubsystem.HangPosition.L3),
                new HangCommand(robot.hang, HangSubsystem.HangPosition.DOWN),
                new HangCommand(robot.hang, HangSubsystem.HangPosition.L3),
                new HangCommand(robot.hang, HangSubsystem.HangPosition.DOWN),
                new HangCommand(robot.hang, HangSubsystem.HangPosition.L3),
                new HangCommand(robot.hang, HangSubsystem.HangPosition.DOWN),
                new HangCommand(robot.hang, HangSubsystem.HangPosition.L3),
                new HangCommand(robot.hang, HangSubsystem.HangPosition.DOWN),
                new HangCommand(robot.hang, HangSubsystem.HangPosition.L3),
                new HangCommand(robot.hang, HangSubsystem.HangPosition.DOWN),
                new HangCommand(robot.hang, HangSubsystem.HangPosition.L3)
                );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
