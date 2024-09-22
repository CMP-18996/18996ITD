package org.firstinspires.ftc.teamcode.test;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.commands.LiftTrapdoorCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp(name="Trapdoor Test")
public class TrapdoorTest extends CommandOpMode {
    IntakeSubsystem intakeSubsystem;
    HardwareMap hardwareMap;
    Subsystems subsystems = Subsystems.INTAKE;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        Robot robot = new Robot(hardwareMap, subsystems);
        super.schedule(
                new LiftTrapdoorCommand(intakeSubsystem, IntakeSubsystem.TrapdoorState.EJECTING),
                new WaitCommand(3000),
                new LiftTrapdoorCommand(intakeSubsystem, IntakeSubsystem.TrapdoorState.TRANSFERRING),
                new WaitCommand(3000),
                new LiftTrapdoorCommand(intakeSubsystem, IntakeSubsystem.TrapdoorState.CLOSED)
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
