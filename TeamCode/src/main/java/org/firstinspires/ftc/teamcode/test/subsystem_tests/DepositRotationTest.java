package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.DepositRotationCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@Disabled
@TeleOp(name = "Deposit Rotator Test")
public class DepositRotationTest extends CommandOpMode {
    Subsystems subsystems = Subsystems.DEPOSIT;
    Robot robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        super.schedule(
                new SequentialCommandGroup(
                        new WaitCommand(2000),
                        new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.DEPOSITING),
                        new WaitCommand(5000),
                        new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY)
                )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
