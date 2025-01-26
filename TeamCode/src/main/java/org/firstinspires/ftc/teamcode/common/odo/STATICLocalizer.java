package org.firstinspires.ftc.teamcode.common.odo;

import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.constants.OTOSConstants;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

public class STATICLocalizer extends Localizer {
    private HardwareMap hardwareMap;
    private Limelight3A limelight;
    private SparkFunOTOS otos;
    private SparkFunOTOS.Pose2D otosPose;
    private SparkFunOTOS.Pose2D otosVel;
    private SparkFunOTOS.Pose2D otosAcc;
    private double previousHeading;
    private double totalHeading;
    private Pose startPose;

    private LocalizationMode localizationMode;

    public enum LocalizationMode {
        PINPOINT,
        OTOS,
        BUCKET,
        SPECIMEN_PICKUP,
        CHAMBER
    }

    public STATICLocalizer(HardwareMap hardwareMap) {this(hardwareMap, LocalizationMode.OTOS, new Pose());}

    public STATICLocalizer(HardwareMap hardwareMap, LocalizationMode localizationMode) {this(hardwareMap, localizationMode, new Pose());}

    public STATICLocalizer(HardwareMap hardwareMap, LocalizationMode localizationMode, Pose startPose) {
        this.hardwareMap = hardwareMap;
        this.localizationMode = localizationMode;

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
            case OTOS:
                Pose pose = new Pose(otosPose.x, otosPose.y, otosPose.h);
                Vector vec = pose.getVector();
                vec.rotateVector(startPose.getHeading());
                return MathFunctions.addPoses(startPose, new Pose(vec.getXComponent(), vec.getYComponent(), pose.getHeading()));
            case PINPOINT:

            case BUCKET:

            case CHAMBER:

            case SPECIMEN_PICKUP:

            default:
                throw new IllegalArgumentException("Unknown localizer mode");
        }
    }

    @Override
    public Pose getVelocity() {
        switch (localizationMode) {
            case OTOS:
                return new Pose(otosVel.x, otosVel.y, otosVel.h);
            case PINPOINT:

            case BUCKET:

            case CHAMBER:

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
        if(localizationMode == LocalizationMode.OTOS) {
            otos.getPosVelAcc(otosPose, otosVel, otosAcc);
            totalHeading += MathFunctions.getSmallestAngleDifference(otosPose.h, previousHeading);
            previousHeading = otosPose.h;
        }
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

    private void configureLimelight() {

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
}
