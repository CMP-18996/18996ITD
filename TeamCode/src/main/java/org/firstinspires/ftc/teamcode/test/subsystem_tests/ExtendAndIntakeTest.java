package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.ExtendAndBeginIntakeCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
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
                new ExtendAndBeginIntakeCommand(robot.extension, robot.intake, robot.lift)
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
