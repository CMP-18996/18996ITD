package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
@TeleOp(name = "Intake Transfer Test")
public class IntakeTransferTest extends CommandOpMode {
    Subsystems subsystems = Subsystems.INTAKE;
    Robot robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        super.schedule(
            new SequentialCommandGroup(
                new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.PICKING_UP),
                new WaitCommand(3000),
                new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                new WaitCommand(3000),
                new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.PICKING_UP),
                new WaitCommand(3000),
                new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.TRANSFERRING)
            )
        );
    }

    @Override
    public void run() {
        telemetry.addData("Intake Rotator State", robot.intake.getIntakeRotatorState());
        telemetry.update();

        CommandScheduler.getInstance().run();
    }
}
