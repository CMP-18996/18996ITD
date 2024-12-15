package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.ExtendAndBeginIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.RetractAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
@Disabled
@TeleOp(name = "Extend and Begin Intake Command")
public class ExtendAndIntakeTest extends CommandOpMode {
    Subsystems intake = Subsystems.INTAKE;
    Subsystems extension = Subsystems.EXTENSION;
    Subsystems lift = Subsystems.LIFT;
    Robot robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, intake, extension, lift);
        super.schedule(
                new SequentialCommandGroup(
                        new ExtendAndBeginIntakeCommand(robot.extension, robot.intake, robot.lift),
                        new WaitCommand(5000),
                        new RetractAndTransferCommand(robot.extension, robot.intake, robot.deposit)
                )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
