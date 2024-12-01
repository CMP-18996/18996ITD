package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.common.commands.HangCommand;
import org.firstinspires.ftc.teamcode.common.commands.SpecimenArmCommand;
import org.firstinspires.ftc.teamcode.common.commands.SpecimenGripperCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

public class SpecimenTest extends CommandOpMode {
    Subsystems subsystems = Subsystems.SPECIMEN;
    Robot robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        super.schedule(
                new SpecimenArmCommand(robot.specimen, SpecimenSubsystem.SpecimenPosition.CHAMBER),
                new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.OPEN),
                new SpecimenArmCommand(robot.specimen, SpecimenSubsystem.SpecimenPosition.REST),
                new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.CLOSED)
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
