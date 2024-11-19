package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.DepositRotationCommand;
import org.firstinspires.ftc.teamcode.common.commands.ExtendCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

import java.security.cert.Extension;

@TeleOp(name = "Extend Command Test")
public class ExtendCommandTest extends CommandOpMode {
    Subsystems subsystems = Subsystems.EXTENSION;
    Robot robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        super.schedule(
                new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.FULLY_EXTENDED),
                new WaitCommand(1000),
                new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.CONTRACTED),
                new WaitCommand(1000),
                new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.HALF_EXTENDED),
                new WaitCommand(1000),
                new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.CONTRACTED)
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}