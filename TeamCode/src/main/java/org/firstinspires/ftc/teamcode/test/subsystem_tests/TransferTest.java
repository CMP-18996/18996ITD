package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.complexCommands.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp(name="Your mom")
public class TransferTest extends CommandOpMode {
    Robot robot;

    public void initialize() {
        robot = new Robot(hardwareMap, Subsystems.ALL);
        CommandScheduler.getInstance().reset();

        CommandScheduler.getInstance().schedule(
                new WaitCommand(3000),
                new TransferSampleCommand(robot.extension, robot.intake, robot.deposit, robot.lift)
        );
    }

    public void run() {
        CommandScheduler.getInstance().run();
    }
}
