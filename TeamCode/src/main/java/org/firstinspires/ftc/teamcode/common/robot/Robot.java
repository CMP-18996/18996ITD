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
    public HardwareMap hardwareMap;
    public DepositSubsystem deposit;
    public ExtensionSubsystem extension;
    public IntakeSubsystem intake;
    public LiftSubsystem lift;
    public SpecimenSubsystem specimen;
    public HangSubsystem hang;

    public Robot(HardwareMap hardwareMap, Subsystems... subsystems) {
        this.hardwareMap = hardwareMap;

        for (Subsystems subsystem : subsystems) {
            switch(subsystem) {
                case ALL:
                    extension = new ExtensionSubsystem(hardwareMap);
                    deposit = new DepositSubsystem(hardwareMap);
                    lift = new LiftSubsystem(hardwareMap);
                    intake = new IntakeSubsystem(hardwareMap);
                    specimen = new SpecimenSubsystem(hardwareMap);
                    hang = new HangSubsystem(hardwareMap);
                case INTAKE:
                    intake = new IntakeSubsystem(hardwareMap);
                case HANG:
                    hang = new HangSubsystem(hardwareMap);
                case LIFT:
                    lift = new LiftSubsystem(hardwareMap);
                case DEPOSIT:
                    deposit = new DepositSubsystem(hardwareMap);
                case SPECIMEN:
                    specimen = new SpecimenSubsystem(hardwareMap);
                case EXTENSION:
                    extension = new ExtensionSubsystem(hardwareMap);
            }
        }
    }

    /*
    public boolean acceptColor(IntakeSubsystem.Color color) {
        if(color.equals(IntakeSubsystem.Color.NONE)) {
            return false;
        }
        else if(color.equals(IntakeSubsystem.Color.RED)) {
            if(team.equals(RED))  {
                return true;
            }
            else{
                return false;
            }
        }
        else if(color.equals(IntakeSubsystem.Color.BLUE)) {
            if(team.equals(BLUE))  {
                return true;
            }
            else{
                return false;
            }
        }
        else {
            return acceptYellow;
        }
    }
    */
}



