package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp(name="Lift Position")
public class LiftPositionTest extends CommandOpMode {

    Subsystems subsystems;
    Robot robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems.LIFT);
        super.schedule(
            new SequentialCommandGroup(
                new LiftSetPosition(robot.lift, robot.lift.GROUND),
                new LiftSetPosition(robot.lift, robot.lift.LOW_RUNG),
                new LiftSetPosition(robot.lift, robot.lift.HIGH_RUNG),
                new LiftSetPosition(robot.lift, robot.lift.GROUND),
                new LiftSetPosition(robot.lift, robot.lift.LOW_BASKET),
                new LiftSetPosition(robot.lift, robot.lift.HIGH_BASKET),
                new LiftSetPosition(robot.lift, robot.lift.GROUND)
            )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
