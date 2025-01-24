package org.firstinspires.ftc.teamcode.common.robot;

import static org.firstinspires.ftc.teamcode.common.robot.Team.BLUE;
import static org.firstinspires.ftc.teamcode.common.robot.Team.RED;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

public class Robot {
    public DepositSubsystem deposit;
    public ExtensionSubsystem extension;
    public IntakeSubsystem intake;
    public LiftSubsystem lift;
    public SpecimenSubsystem specimen;
    public HardwareMap hardwareMap;
    private boolean transferring = false;
    private final Team team;
    private boolean acceptYellow = false;

    @Deprecated
    public Robot(HardwareMap hardwareMap, Subsystems... subsystems) {
        this.hardwareMap = hardwareMap;
        this.team = null;
        for (Subsystems subsystem : subsystems) {
            if (subsystem == Subsystems.ALL) {
                extension = new ExtensionSubsystem(hardwareMap);
                deposit = new DepositSubsystem(hardwareMap);
                lift = new LiftSubsystem(hardwareMap);
                intake = new IntakeSubsystem(hardwareMap);
                specimen = new SpecimenSubsystem(hardwareMap);
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

    public Robot(HardwareMap hardwareMap, Team team, Subsystems... subsystems) {
        this.hardwareMap = hardwareMap;
        this.team = team;
        for (Subsystems subsystem : subsystems) {
            if (subsystem == Subsystems.ALL) {
                extension = new ExtensionSubsystem(hardwareMap);
                deposit = new DepositSubsystem(hardwareMap);
                lift = new LiftSubsystem(hardwareMap);
                intake = new IntakeSubsystem(hardwareMap);
                specimen = new SpecimenSubsystem(hardwareMap);
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

    public Robot(HardwareMap hardwareMap, Team team, boolean acceptYellow, Subsystems... subsystems) {
        this.hardwareMap = hardwareMap;
        this.team = team;
        this.acceptYellow = acceptYellow;

        for (Subsystems subsystem : subsystems) {
            if (subsystem == Subsystems.ALL) {
                extension = new ExtensionSubsystem(hardwareMap);
                deposit = new DepositSubsystem(hardwareMap);
                lift = new LiftSubsystem(hardwareMap);
                intake = new IntakeSubsystem(hardwareMap);
                specimen = new SpecimenSubsystem(hardwareMap);
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

    public void setTransferringState(boolean state) {
        transferring = state;
    }

    public boolean isTransferring() {
        return transferring;
    }

    public boolean acceptColor(IntakeSubsystem.Color color) {
        if(color.equals(IntakeSubsystem.Color.NONE)) {
            return false;
        }
        else if(color.equals(IntakeSubsystem.Color.RED) && team.equals(RED)) {
            return true;
        }
        else if(color.equals(IntakeSubsystem.Color.BLUE) && team.equals(BLUE)) {
            return true;
        }
        else {
            return acceptYellow;
        }
    }

    public Team getTeam() {
        return team;
    }
}



