package org.firstinspires.ftc.teamcode.common.odo;

import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.constants.OTOSConstants;
import com.pedropathing.localization.localizers.OTOSLocalizer;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

public class STATICLocalizer extends Localizer {
    private HardwareMap hardwareMap;

    private Localizer pinpointLocalizer;
    private Localizer otosLocalizer;

    public AnalogInput rightSideUltrasonic;
    public AnalogInput backSideUltrasonic;
    public AnalogInput backLeftAngledUltrasonic;
    public AnalogInput backRightAngledUltrasonic;

    private LocalizationMode localizationMode;

    public enum LocalizationMode {
        NORMAL,
        SAMPLE_DEPOSIT,
        SPECIMEN_PICKUP,
        SPECIMEN_DEPOSIT
    }

    public STATICLocalizer(HardwareMap hardwareMap) {this(hardwareMap, LocalizationMode.NORMAL, new Pose());}

    public STATICLocalizer(HardwareMap hardwareMap, LocalizationMode localizationMode) {this(hardwareMap, localizationMode, new Pose());}

    public STATICLocalizer(HardwareMap hardwareMap, LocalizationMode localizationMode, Pose startPose) {
        this.hardwareMap = hardwareMap;
        this.localizationMode = localizationMode;
        this.pinpointLocalizer = new PinpointLocalizer(hardwareMap, startPose);
        this.otosLocalizer = new OTOSLocalizer(hardwareMap, startPose);

        rightSideUltrasonic = hardwareMap.get(AnalogInput.class, HardwareMapNames.RIGHT_SIDE_ULTRASONIC);
        backSideUltrasonic =  hardwareMap.get(AnalogInput.class, HardwareMapNames.BACK_SIDE_ULTRASONIC);
        backLeftAngledUltrasonic =  hardwareMap.get(AnalogInput.class, HardwareMapNames.BACK_LEFT_SIDE_ULTRASONIC);
        backRightAngledUltrasonic =  hardwareMap.get(AnalogInput.class, HardwareMapNames.BACK_RIGHT_SIDE_ULTRASONIC);
    }

    @Override
    public Pose getPose() {
        if(pinpointLocalizer.isNAN()) {
            return otosLocalizer.getPose();
        }
        return pinpointLocalizer.getPose();
    }

    @Override
    public Pose getVelocity() {
        if(pinpointLocalizer.isNAN()) {
            return otosLocalizer.getVelocity();
        }
        return pinpointLocalizer.getVelocity();
    }

    @Override
    public Vector getVelocityVector() {
        if(pinpointLocalizer.isNAN()) {
            return otosLocalizer.getVelocityVector();
        }
        return pinpointLocalizer.getVelocityVector();
    }

    @Override
    public void setStartPose(Pose startPose) {
        pinpointLocalizer.setStartPose(startPose);
        otosLocalizer.setStartPose(startPose);
    }

    @Override
    public void setPose(Pose setPose) {
        pinpointLocalizer.setPose(setPose);
        otosLocalizer.setPose(setPose);
    }

    @Override
    public void update() {
        if(pinpointLocalizer.isNAN()) {
            otosLocalizer.update();
        }
        else {
            pinpointLocalizer.update();
        }
    }

    @Override
    public double getTotalHeading() {
        if(pinpointLocalizer.isNAN()) {
            return otosLocalizer.getTotalHeading();
        }
        return pinpointLocalizer.getTotalHeading();
    }

    @Override
    public boolean isNAN() { return false; }

    public boolean usingPinpoint() {
        return !pinpointLocalizer.isNAN();
    }

    public void setLocalizationMode(LocalizationMode localizationMode) {
        this.localizationMode = localizationMode;
    }

    public LocalizationMode getLocalizationMode() {
        return localizationMode;
    }

    /** Irrelevant to this localizer. */
    @Override
    public double getForwardMultiplier() {return 0;}
    @Override
    public double getLateralMultiplier() {return 0;}
    @Override
    public double getTurningMultiplier() {return 0;}
    @Override
    public void resetIMU() {}

    public static double getDistanceFromVoltage(double voltage) {
        return 3300 * 520 / voltage;
    }

    public Pose measureSampleStartPose() {
        return new Pose(144 - getDistanceFromVoltage(backSideUltrasonic.getVoltage()) - 9, 9, -90);
    }

    public Pose measureSpecimenStartPose() {
        return new Pose(getDistanceFromVoltage(rightSideUltrasonic.getVoltage()) + 8, 15.5/2, 0);
    }
}
