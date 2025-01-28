package org.firstinspires.ftc.teamcode.common.odo;

import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.constants.OTOSConstants;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

public class STATICLocalizer extends Localizer {
    private HardwareMap hardwareMap;
    private Pose startPose;
    private SparkFunOTOS otos;
    private SparkFunOTOS.Pose2D otosPose;
    private SparkFunOTOS.Pose2D otosVel;
    private SparkFunOTOS.Pose2D otosAcc;
    private double previousHeading;
    private double totalHeading;

    private AnalogInputController analogInputController;
    private AnalogInput rightSideUltrasonic;
    private AnalogInput backSideUltrasonic;
    private AnalogInput backLeftAngledUltrasonic;
    private AnalogInput backRightAngledUltrasonic;

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

        rightSideUltrasonic = hardwareMap.get(AnalogInput.class, HardwareMapNames.RIGHT_SIDE_ULTRASONIC);
        //backSideUltrasonic =  hardwareMap.get(AnalogInput.class, HardwareMapNames.BACK_SIDE_ULTRASONIC);
        //backLeftAngledUltrasonic =  hardwareMap.get(AnalogInput.class, HardwareMapNames.BACK_LEFT_SIDE_ULTRASONIC);
        //backRightAngledUltrasonic =  hardwareMap.get(AnalogInput.class, HardwareMapNames.BACK_RIGHT_SIDE_ULTRASONIC);

        // TODO: eliminate this
        if (OTOSConstants.useCorrectedOTOSClass) {
            otos = this.hardwareMap.get(SparkFunOTOSCorrected.class, OTOSConstants.hardwareMapName);
        } else {
            otos = this.hardwareMap.get(SparkFunOTOS.class, OTOSConstants.hardwareMapName);
        }

        configureOTOS();

        otos.calibrateImu();
        otos.resetTracking();

        setStartPose(startPose);

        otosPose = new SparkFunOTOS.Pose2D();
        otosVel = new SparkFunOTOS.Pose2D();
        otosAcc = new SparkFunOTOS.Pose2D();

        totalHeading = 0.0;
        previousHeading = startPose.getHeading();

        otos.resetTracking();
    }

    @Override
    public Pose getPose() {
        switch (localizationMode) {
            case NORMAL:
                Pose pose = new Pose(otosPose.x, otosPose.y, otosPose.h);
                Vector vec = pose.getVector();
                vec.rotateVector(startPose.getHeading());
                return MathFunctions.addPoses(startPose, new Pose(vec.getXComponent(), vec.getYComponent(), pose.getHeading()));
            case SAMPLE_DEPOSIT:

            case SPECIMEN_DEPOSIT:

            case SPECIMEN_PICKUP:

            default:
                throw new IllegalArgumentException("Unknown localizer mode");
        }
    }

    @Override
    public Pose getVelocity() {
        switch (localizationMode) {
            case NORMAL:
                return new Pose(otosVel.x, otosVel.y, otosVel.h);
            case SAMPLE_DEPOSIT:

            case SPECIMEN_DEPOSIT:

            case SPECIMEN_PICKUP:

            default:
                throw new IllegalArgumentException("Unknown localizer mode");
        }
    }

    @Override
    public Vector getVelocityVector() {
        return getVelocity().getVector();
    }

    @Override
    public void setStartPose(Pose startPose) {
        this.startPose = startPose;
    }

    @Override
    public void setPose(Pose setPose) {
        otos.resetTracking();

        Pose setOTOSPose = MathFunctions.subtractPoses(setPose, startPose);
        otos.setPosition(new SparkFunOTOS.Pose2D(setOTOSPose.getX(), setOTOSPose.getY(), setOTOSPose.getHeading()));
    }

    @Override
    public void update() {
        otos.getPosVelAcc(otosPose, otosVel, otosAcc);
        totalHeading += MathFunctions.getSmallestAngleDifference(otosPose.h, previousHeading);
        previousHeading = otosPose.h;
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    private void configureOTOS() {
        otos.setLinearUnit(OTOSConstants.linearUnit);
        otos.setAngularUnit(OTOSConstants.angleUnit);
        otos.setOffset(OTOSConstants.offset);
        otos.setLinearScalar(OTOSConstants.linearScalar);
        otos.setAngularScalar(OTOSConstants.angularScalar);
    }

    public void setLocalizationMode(LocalizationMode localizationMode) {
        this.localizationMode = localizationMode;
        update(); // TODO: make sure this is necessary
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

    private static double getDistanceFromVoltage(double voltage) {
        return 3300 * 520 / voltage;
    }

    public Pose measureSampleStartPose() {
        return new Pose(144 - getDistanceFromVoltage(backSideUltrasonic.getVoltage()) - 9, 9, -90);
    }

    public Pose measureSpecimenStartPose() {
        return new Pose(getDistanceFromVoltage(rightSideUltrasonic.getVoltage()) + 9, 9, 0);
    }
}
