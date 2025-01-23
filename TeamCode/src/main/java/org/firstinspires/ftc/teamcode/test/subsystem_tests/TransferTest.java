package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.complexCommands.ExtendAndBeginIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.RetractAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.WaitForColorCommand;
import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
@TeleOp(name = "Transfer Test")
public class TransferTest extends CommandOpMode {
    Subsystems intake = Subsystems.INTAKE;
    Subsystems deposit = Subsystems.DEPOSIT;
    Subsystems lift = Subsystems.LIFT;
    Subsystems extension = Subsystems.EXTENSION;
    Robot robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, intake, deposit, lift, extension);

        super.schedule(
                new ExtendAndBeginIntakeCommand(robot.extension, robot.intake),
                new WaitForColorCommand(robot.intake, IntakeSubsystem.Color.YELLOW),
                new RetractAndTransferCommand(robot.extension, robot.intake, robot.deposit, robot.lift),
                new LiftSetPosition(robot.lift, LiftSubsystem.LiftState.HIGH_BUCKET),
                new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.READY),
                new WaitCommand(1000),
                new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.DEPOSIT)
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        telemetry.update();
    }
}
