package org.firstinspires.ftc.teamcode.common.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;

public class ZeroLiftCommand {
    public ExtensionSubsystem extensionSubsystem;
    public LiftSubsystem liftSubsystem;
    ElapsedTime timer = new ElapsedTime();

    public ZeroLiftCommand(ExtensionSubsystem extensionSubsystem, LiftSubsystem liftSubsystem) {
        this.extensionSubsystem = extensionSubsystem;
        this.liftSubsystem = liftSubsystem;
    }

//    public
}
