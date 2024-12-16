package org.firstinspires.ftc.teamcode.common.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

public class Robot {
    public DepositSubsystem deposit;
    public ExtensionSubsystem extension;
    public HangSubsystem hang;
    public IntakeSubsystem intake;
    public LiftSubsystem lift;
    public SpecimenSubsystem specimen;
    public HardwareMap hardwareMap;
    private boolean transferring = false;
    private final Team team;

    @Deprecated
    public Robot(HardwareMap hardwareMap, Subsystems... subsystems) {
        this.hardwareMap = hardwareMap;
        this.team = null;
        for (Subsystems subsystem : subsystems) {
            if (subsystem == Subsystems.ALL) {
                //hang = new HangSubsystem(hardwareMap);
                extension = new ExtensionSubsystem(hardwareMap);
                deposit = new DepositSubsystem(hardwareMap);
                lift = new LiftSubsystem(hardwareMap);
                intake = new IntakeSubsystem(hardwareMap);
                specimen = new SpecimenSubsystem(hardwareMap);
            }
            //else if (subsystem == Subsystems.HANG) {
            //    hang = new HangSubsystem(hardwareMap);
            //}
            else if (subsystem == Subsystems.EXTENSION) {
                extension = new ExtensionSubsystem(hardwareMap);
            }
            else if (subsystem == Subsystems.DEPOSIT) {
                deposit = new DepositSubsystem(hardwareMap);
            }
            else if (subsystem == Subsystems.LIFT) {
                lift = new LiftSubsystem(hardwareMap);
            }
            else if (subsystem == Subsystems.SPECIMEN) {
                specimen = new SpecimenSubsystem(hardwareMap);
            }
            else if (subsystem == Subsystems.INTAKE) {
                intake = new IntakeSubsystem(hardwareMap);
            }
        }
    }

    public Robot(HardwareMap hardwareMap, Team team, Subsystems... subsystems) {
        this.hardwareMap = hardwareMap;
        this.team = team;
        for (Subsystems subsystem : subsystems) {
            if (subsystem == Subsystems.ALL) {
                //hang = new HangSubsystem(hardwareMap);
                extension = new ExtensionSubsystem(hardwareMap);
                deposit = new DepositSubsystem(hardwareMap);
                lift = new LiftSubsystem(hardwareMap);
                intake = new IntakeSubsystem(hardwareMap);
                specimen = new SpecimenSubsystem(hardwareMap);
            }
            else if (subsystem == Subsystems.HANG) {
                //hang = new HangSubsystem(hardwareMap);
            }
            else if (subsystem == Subsystems.EXTENSION) {
                extension = new ExtensionSubsystem(hardwareMap);
            }
            else if (subsystem == Subsystems.DEPOSIT) {
                deposit = new DepositSubsystem(hardwareMap);
            }
            else if (subsystem == Subsystems.LIFT) {
                lift = new LiftSubsystem(hardwareMap);
            }
            else if (subsystem == Subsystems.SPECIMEN) {
                specimen = new SpecimenSubsystem(hardwareMap);
            }
            else if (subsystem == Subsystems.INTAKE) {
                intake = new IntakeSubsystem(hardwareMap);
            }
        }
    }

    public Robot(HardwareMap hardwareMap, Team team, boolean TeleOp, Subsystems... subsystems) {
        this.hardwareMap = hardwareMap;
        this.team = team;
        for (Subsystems subsystem : subsystems) {
            if (subsystem == Subsystems.ALL) {
                //hang = new HangSubsystem(hardwareMap);
                extension = new ExtensionSubsystem(hardwareMap, TeleOp);
                deposit = new DepositSubsystem(hardwareMap);
                lift = new LiftSubsystem(hardwareMap, TeleOp);
                intake = new IntakeSubsystem(hardwareMap);
                specimen = new SpecimenSubsystem(hardwareMap);
            }
            else if (subsystem == Subsystems.HANG) {
                //hang = new HangSubsystem(hardwareMap);
            }
            else if (subsystem == Subsystems.EXTENSION) {
                extension = new ExtensionSubsystem(hardwareMap, TeleOp);
            }
            else if (subsystem == Subsystems.DEPOSIT) {
                deposit = new DepositSubsystem(hardwareMap);
            }
            else if (subsystem == Subsystems.LIFT) {
                lift = new LiftSubsystem(hardwareMap, TeleOp);
            }
            else if (subsystem == Subsystems.SPECIMEN) {
                specimen = new SpecimenSubsystem(hardwareMap);
            }
            else if (subsystem == Subsystems.INTAKE) {
                intake = new IntakeSubsystem(hardwareMap);
            }
        }
    }

    public void setTransferringState(boolean state) {
        transferring = state;
    }

    public boolean isTransferring() {
        return transferring;
    }

    public Team getTeam() {
        return team;
    }
}



