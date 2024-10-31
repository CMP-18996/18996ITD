package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    // Constants
    final static double TRAPDOOR_MIN_ROT = 0.0;
    final static double TRAPDOOR_MAX_ROT = 0.0;
    final static double INTAKE_ROTATION_MIN_ROT = 0.0; // max and min rotation used as what arm is actually being rotated to, subject to change
    final static double INTAKE_ROTATION_MAX_ROT = 0.0;
    public static double CLOSED_VALUE = .5;
    public static double EJECTING_VALUE = 1.0;
    public static double TRANSFERRING_VALUE = 0;
    public static double ACTIVE_VALUE = 1.00;
    public static double DISABLED_VALUE = 0.0;

    // State
    final private CRServoImpl intakeServo1;
    final private CRServoImpl intakeServo2;
//     final private ServoEx trapdoorServo, intakeRotationServo;
    final private ServoEx trapdoorServo;
    public ColorSensor colorSensor; //TODO: isn't this and the above INTERFACES? you can't instantiate interfaces...
    private TrapdoorState trapdoorState = TrapdoorState.CLOSED;
    private IntakingState intakingState = IntakingState.DISABLED;
    private IntakeRotatorState intakeRotatorState = IntakeRotatorState.TRANSFERRING;
    public ColorState colorState = ColorState.NONE;
    public enum TrapdoorState {
        CLOSED(CLOSED_VALUE),
        EJECTING(EJECTING_VALUE);
        public double val;
        TrapdoorState(double inval) {val = inval;}
    }
    public enum IntakingState {
        ACTIVE(ACTIVE_VALUE),
        DISABLED(DISABLED_VALUE);
        public double val;
        IntakingState(double inval) {val = inval;}
    }

    public enum IntakeRotatorState {
        TRANSFERRING(INTAKE_ROTATION_MIN_ROT),
        DROPPING(INTAKE_ROTATION_MAX_ROT);
        public double val;
        IntakeRotatorState(double inval) {val = inval;}
    }
    public enum ColorState {
        NONE("None"),
        YELLOW("Yellow"),
        RED("Red"),
        BLUE("Blue");
        public String color;
        ColorState(String incolor) {color = incolor;}
        public String toString() {return color;}

    }
    public String updateColorState(){
        int r, g, b;
        r = colorSensor.red();
        g = colorSensor.green();
        b = colorSensor.blue();
        if(r > g && r > b){
            colorState = ColorState.RED;
        }
        else if(g > r && g > b){
            colorState = ColorState.YELLOW;
        }
        else if(b > r && b > g){
            colorState = ColorState.BLUE;
        }
        else{
            colorState = ColorState.NONE;
        }
        return colorState.toString();
    }
    public void updateTrapdoorState(TrapdoorState setState) {
        trapdoorState = setState;
        trapdoorServo.setPosition(trapdoorState.val);
    }

    public void updateIntakingState(IntakingState setState) {
        intakingState = setState;
        intakeServo1.setPower(intakingState.val);
        intakeServo2.setPower(intakingState.val);
    }

    public void updateIntakeRotatorState(IntakeRotatorState setState) {
        intakeRotatorState = setState;
        // intakeRotationServo.setPosition(intakeRotatorState.val);
    }

    public TrapdoorState getTrapdoorState() {
        return trapdoorState;
    }

    public IntakingState getIntakingState() {
        return intakingState;
    }

    public IntakeRotatorState getIntakeRotatorState() {
        return intakeRotatorState;
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {
         trapdoorServo = new SimpleServo(hardwareMap, "trapdoor", TRAPDOOR_MIN_ROT, TRAPDOOR_MAX_ROT);
        // intakeRotationServo = new SimpleServo(hardwareMap, "intakeRotator", INTAKE_ROTATION_MIN_ROT, INTAKE_ROTATION_MAX_ROT);
        intakeServo1 = hardwareMap.get(CRServoImpl.class, "intake1");
        intakeServo2 = hardwareMap.get(CRServoImpl.class, "intake2");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        intakeServo1.setDirection(DcMotorSimple.Direction.FORWARD); // subject to change
        intakeServo2.setDirection(DcMotorSimple.Direction.REVERSE); // subject to change
    }
}
