package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.ExtendCommand;
import org.firstinspires.ftc.teamcode.common.commands.ExtensionMotorPowerCommand;
import org.firstinspires.ftc.teamcode.common.commands.ExtensionPositionCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

import java.security.cert.Extension;

@Disabled
@TeleOp(name="Extension Power Position")
public class ExtensionPowerAndPositionTest extends CommandOpMode {
    Subsystems subsystems = Subsystems.EXTENSION;
    Robot robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        super.schedule(
            new ParallelCommandGroup (
                new ExtensionPositionCommand(robot.extension, 500),
                new ExtensionMotorPowerCommand(robot.extension, 0.2)
            ),
            new WaitCommand(2000),
            new ParallelCommandGroup (
                new ExtensionPositionCommand(robot.extension, 0),
                new ExtensionMotorPowerCommand(robot.extension, 0.6)
            )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
