package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.HangCommand;
import org.firstinspires.ftc.teamcode.common.commands.SpecimenArmCommand;
import org.firstinspires.ftc.teamcode.common.commands.SpecimenGripperCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.teleops.SPECIKEMARMTET;

@TeleOp(name =  "MLK Jr")
@Config
public class SpecimenTest extends CommandOpMode {
    Subsystems subsystems = Subsystems.SPECIMEN;
    Robot robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        super.schedule(
                new SpecimenArmCommand(robot.specimen, SpecimenSubsystem.SpecimenPosition.REST),
                new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.CLOSED),
                new WaitCommand(1000),
                new SpecimenArmCommand(robot.specimen, SpecimenSubsystem.SpecimenPosition.WALL),
                new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.OPEN),
                new WaitCommand(1000),
                new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.CLOSED),
                new WaitCommand(100),
                new SpecimenArmCommand(robot.specimen, SpecimenSubsystem.SpecimenPosition.CHAMBER),
                new WaitCommand(1000),
                new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.OPEN),
                new WaitCommand(50),
                new SpecimenArmCommand(robot.specimen, SpecimenSubsystem.SpecimenPosition.REST),
                new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.CLOSED)
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("POWER", robot.specimen.power);
        telemetry.addData("ERR", robot.specimen.error);
        telemetry.addData("POSITIIN", robot.specimen.specimenPosition);
        telemetry.addData("GRI", robot.specimen.gripperPosition);
        telemetry.update();
    }
}
