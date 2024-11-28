package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp(name="Lift Position")
public class LiftPositionTest extends CommandOpMode {
    Robot robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, Subsystems.LIFT);
        CommandScheduler.getInstance().registerSubsystem(robot.lift);
        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                new LiftSetPosition(robot.lift, robot.lift.GROUND),
                    new WaitCommand(3000),
                new LiftSetPosition(robot.lift, robot.lift.GROUND),
                    new WaitCommand(3000),
                new LiftSetPosition(robot.lift, robot.lift.LOW_BASKET),
                    new WaitCommand(3000),
                new LiftSetPosition(robot.lift, robot.lift.HIGH_BASKET),
                    new WaitCommand(3000),
                new LiftSetPosition(robot.lift, robot.lift.GROUND),
                    new WaitCommand(3000)
            )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("Target:", robot.lift.getCurrTarget());
        telemetry.addData("Current Position", robot.lift.getCurrentPosition());
        telemetry.update();
    }
}
