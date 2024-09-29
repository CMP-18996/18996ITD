package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class LiftSetPosition extends CommandBase {
    LiftSubsystem liftSubsystem;
    int target;
        public LiftSetPosition(LiftSubsystem liftSubsystem, int targetPosition){
            target = targetPosition;
            this.liftSubsystem = liftSubsystem;
            addRequirements(liftSubsystem);
            }

        @Override
        public void initialize() {
            liftSubsystem.setTargetPosition(target);
        }

        @Override
        public boolean isFinished() {
            return liftSubsystem.motorWorking();
        }
}
