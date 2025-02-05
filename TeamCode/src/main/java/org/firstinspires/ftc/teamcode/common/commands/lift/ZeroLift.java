package org.firstinspires.ftc.teamcode.common.commands.lift;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class ZeroLift extends SequentialCommandGroup {
    public ZeroLift(LiftSubsystem liftSubsystem) {
        addCommands(
                new SequentialCommandGroup(
                        new LiftSetPosition_INST(liftSubsystem, LiftSubsystem.LiftState.ZEROING),
                        new WaitCommand(100),
                        new InstantCommand(liftSubsystem::resetEncoders),
                        new LiftSetPosition_INST(liftSubsystem, LiftSubsystem.LiftState.TRANSFER)
                )
        );
    }
}
