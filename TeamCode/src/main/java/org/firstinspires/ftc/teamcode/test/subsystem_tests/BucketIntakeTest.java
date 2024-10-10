package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
@TeleOp(name = "Bucket Intake Test")
public class BucketIntakeTest extends CommandOpMode {
    IntakeSubsystem intakeSubsystem;
    HardwareMap hardwareMap;
    Subsystems subsystems = Subsystems.INTAKE;
    Robot robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        super.schedule(
                new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.ACTIVE),
                new WaitCommand(3000),
                new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.DISABLED),
                new WaitCommand(3000),
                new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.ACTIVE),
                new WaitCommand(3000),
                new IntakeCommand(intakeSubsystem, IntakeSubsystem.IntakingState.DISABLED)
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
