package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

public class LiftPositionTest extends CommandOpMode {
    LiftSubsystem liftSubsystem;
    Subsystems subsystems;
    Robot robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems.LIFT);
        super.schedule(
            new SequentialCommandGroup(
                new LiftSetPosition(liftSubsystem, liftSubsystem.GROUND),
                new LiftSetPosition(liftSubsystem, liftSubsystem.LOW_RUNG),
                new LiftSetPosition(liftSubsystem, liftSubsystem.HIGH_RUNG),
                new LiftSetPosition(liftSubsystem, liftSubsystem.GROUND),
                new LiftSetPosition(liftSubsystem, liftSubsystem.LOW_BASKET),
                new LiftSetPosition(liftSubsystem, liftSubsystem.HIGH_BASKET),
                new LiftSetPosition(liftSubsystem, liftSubsystem.GROUND)
            )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
