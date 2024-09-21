package org.firstinspires.ftc.teamcode.common.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

public class Robot {
    public DepositSubsystem deposit;
    public ExtensionSubsystem extension;
    public HangSubsystem hang;
    public IntakeSubsystem intake;
    public LiftSubsystem lift;
    public HardwareMap hardwareMap;

    public Robot(HardwareMap hardwareMap, Subsystems... subsystems) {
        this.hardwareMap = hardwareMap;
        for (Subsystems subsystem : subsystems) {
            if (subsystem == Subsystems.ALL) {
                hang = new HangSubsystem();
                extension = new ExtensionSubsystem();
                deposit = new DepositSubsystem();
                lift = new LiftSubsystem();
                intake = new IntakeSubsystem();
            }
            else if (subsystem == Subsystems.HANG) {
                hang = new HangSubsystem();
            }
            else if (subsystem == Subsystems.EXTENSION) {
                extension = new ExtensionSubsystem();
            }
            else if (subsystem == Subsystems.DEPOSIT) {
                deposit = new DepositSubsystem();
            }
            else if (subsystem == Subsystems.LIFT) {
                lift = new LiftSubsystem();
            }
            else if (subsystem == Subsystems.INTAKE) {
                intake = new IntakeSubsystem();
            }
        }
    }
}
