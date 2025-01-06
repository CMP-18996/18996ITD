package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.ExtendAndBeginIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.RetractAndTransferClawsCommand;
import org.firstinspires.ftc.teamcode.common.commands.RetractAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
@TeleOp(name = "Extend and Begin Intake Command")
public class ExtendAndIntakeTest extends CommandOpMode {
    Subsystems intake = Subsystems.INTAKE;
    Subsystems extension = Subsystems.EXTENSION;
    Subsystems lift = Subsystems.LIFT;
    Subsystems deposit = Subsystems.DEPOSIT;
    Robot robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, intake, extension, lift, deposit);
        super.schedule(
                new SequentialCommandGroup(
                        new ExtendAndBeginIntakeCommand(robot.extension, robot.intake, robot.lift),
                        new WaitCommand(1000),
                        new RetractAndTransferClawsCommand(robot.extension, robot.intake, robot.deposit, robot.lift)
                )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
