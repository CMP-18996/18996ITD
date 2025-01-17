package org.firstinspires.ftc.teamcode.common.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;
import org.firstinspires.ftc.teamcode.common.robot.Team;

@Config
public class IntakeSubsystem extends SubsystemBase {
    // Constants
    public static double INTAKE_ROTATION_TRANSFER = 0.37; // max and min rotation used as what arm is actually being rotated to, subject to change
    public static double INTAKE_ROTATION_PICK_UP = 0.97;
    public static double INTAKE_ROTATION_MOVING = 0.6;
    public static double INTAKE_PIVOT_PICK_UP = .4;
    public static double INTAKE_PIVOT_MOVING = .3;
    public static double INTAKE_PIVOT_DROPPING = .2;
    public static double CLOSED_VALUE = .5;
    public static double EJECTING_VALUE = 1.0;
    public static double ACTIVE_VALUE = 1.0;
    public static double DISABLED_VALUE = 0.0;
    public static double REVERSING_VALUE = -1.0;

    // State
    final private CRServoImpl intakeServo1;
//     final private ServoEx trapdoorServo, intakeRotationServo;
    final private Servo trapdoorServo, intakeRotationServo, intakePivotServo;
    public ColorSensor colorSensor; //TODO: isn't this and the above INTERFACES? you can't instantiate interfaces...
    private TrapdoorState trapdoorState = TrapdoorState.CLOSED;
    private IntakingState intakingState = IntakingState.DISABLED;
    private IntakeRotatorState intakeRotatorState = IntakeRotatorState.TRANSFERRING;
    private IntakeArmPivotState intakeArmPivotState;
    public ColorState colorState = ColorState.NONE;
    public ColorSensorState colorSensorState = ColorSensorState.WORKING;
    public enum TrapdoorState {
        CLOSED(CLOSED_VALUE),
        EJECTING(EJECTING_VALUE);
        public double val;
        TrapdoorState(double inval) {val = inval;}
    }
    public enum IntakingState {
        ACTIVE(ACTIVE_VALUE),
        DISABLED(DISABLED_VALUE),
        REVERSING(REVERSING_VALUE);
        public double val;
        IntakingState(double inval) {val = inval;}
    }
    public enum IntakeArmPivotState {
        PICK_UP(INTAKE_PIVOT_PICK_UP),
        MOVING(INTAKE_PIVOT_MOVING),
        DROPPING(INTAKE_PIVOT_DROPPING);
        public double val;
        IntakeArmPivotState(double inval) {val = inval; }
    }


    public enum IntakeRotatorState {
        TRANSFERRING(INTAKE_ROTATION_TRANSFER),
        MOVING(INTAKE_ROTATION_MOVING),
        PICKING_UP(INTAKE_ROTATION_PICK_UP);
        public double val;
        IntakeRotatorState(double inval) {val = inval;}
    }
    public enum ColorState {
        NONE,
        YELLOW,
        RED,
        BLUE
    }

    public enum ColorSensorState {
        WORKING,
        BROKEN
    }

    public void slightlyIncrementRotator(double increment) {
        intakeRotationServo.setPosition(Range.clip(intakeRotatorState.val + increment, -1.0, 1.0));
    }
    public ColorState updateColorState(){
        int r, g, b, a;
        r = colorSensor.red();
        g = colorSensor.green();
        b = colorSensor.blue();
        a = colorSensor.alpha();
        if (a < 100) return ColorState.NONE;

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
            // this should NOT be NONE
            // this is a edge case that we have to deal with
        }
        return colorState;
    }

    public Team updateColorState2(){
        int r, g, b, a;
        r = colorSensor.red();
        g = colorSensor.green();
        b = colorSensor.blue();
        a = colorSensor.alpha();
        Team returnedColorState;

        if (colorSensorState == ColorSensorState.BROKEN) {
            colorState = ColorState.NONE;

            return Team.NONE;
        }

        if (a < 60) {
            colorState = ColorState.NONE;
            return Team.NONE;
        }

        if(r > g && r > b){
            colorState = ColorState.RED;
            returnedColorState = Team.RED;
        }
        else if(g > r && g > b){
            colorState = ColorState.YELLOW;
            returnedColorState = Team.YELLOW;
        }
        else if(b > r && b > g){
            colorState = ColorState.BLUE;
            returnedColorState = Team.BLUE;
        }
        else{
            colorState = ColorState.NONE;
            returnedColorState = Team.NONE;
        }
        return returnedColorState;
    }

    public void updateColorSensorState(ColorSensorState colorSensorState) {
        this.colorSensorState = colorSensorState;
    }

    public ColorSensorState getColorSensorState() {
        return colorSensorState;
    }

    public void updateTrapdoorState(TrapdoorState setState) {
        trapdoorState = setState;
        trapdoorServo.setPosition(trapdoorState.val);
    }

    public void updateIntakeArmPivotState(IntakeArmPivotState intakeArmPivotState) {
        this.intakeArmPivotState = intakeArmPivotState;
        intakePivotServo.setPosition(intakeArmPivotState.val);
    }

    public void updateIntakingState(IntakingState setState) {
        intakingState = setState;
        intakeServo1.setPower(intakingState.val);
    }

    public void updateIntakeRotatorState(IntakeRotatorState setState) {
        intakeRotatorState = setState;
        intakeRotationServo.setPosition(intakeRotatorState.val);
    }

    public TrapdoorState getTrapdoorState() {
        return trapdoorState;
    }

    public IntakingState getIntakingState() {
        return intakingState;
    }

    public IntakeArmPivotState getIntakeArmPivotState() { return intakeArmPivotState; }

    public IntakeRotatorState getIntakeRotatorState() {
        return intakeRotatorState;
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {
        trapdoorServo = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_TRAPDOOR);
        intakeRotationServo = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_ROTATOR);
        intakePivotServo = hardwareMap.get(Servo.class, HardwareMapNames.INTAKE_ARM_PIVOT);
        intakeServo1 = hardwareMap.get(CRServoImpl.class, HardwareMapNames.INTAKE_SERVO_1);
        colorSensor = hardwareMap.get(ColorSensor.class, HardwareMapNames.INTAKE_COLOR_SENSOR);
        intakeRotationServo.setDirection(Servo.Direction.FORWARD);
        intakeServo1.setDirection(DcMotorSimple.Direction.FORWARD); // subject to change
        this.updateIntakeRotatorState(IntakeRotatorState.TRANSFERRING);
        this.updateIntakingState(IntakingState.DISABLED);
        updateTrapdoorState(TrapdoorState.CLOSED);
    }
}
