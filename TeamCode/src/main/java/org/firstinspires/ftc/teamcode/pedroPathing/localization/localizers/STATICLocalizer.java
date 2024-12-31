package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.odo.OdometryFusion;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

public class STATICLocalizer extends Localizer {
    private OdometryFusion odometryFusion;
    private Pose startPose;
    private Pose2D pos;
    private Pose2D vel;
    //private Pose2D acc;
    private double previousHeading;
    private double totalHeading;

    public STATICLocalizer(OdometryFusion odometryFusion) {
        this(odometryFusion, new Pose());
    }

    public STATICLocalizer(OdometryFusion odometryFusion, Pose startPose) {
        this.odometryFusion = odometryFusion;

        setStartPose(startPose);

        pos = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
        vel = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
        totalHeading = 0;
        previousHeading = startPose.getHeading();

        //reset
    }

    public Pose getPose() {
        Pose pose = new Pose(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.RADIANS));

        Vector vec = pose.getVector();
        vec.rotateVector(startPose.getHeading());

        return MathFunctions.addPoses(startPose, new Pose(vec.getXComponent(), vec.getYComponent(), pose.getHeading()));
    }

    public Pose getVelocity() {
        return new Pose(vel.getX(DistanceUnit.INCH), vel.getY(DistanceUnit.INCH), vel.getHeading(AngleUnit.RADIANS));
    }

    public Vector getVelocityVector() {
        return getVelocity().getVector();
    }

    public void setStartPose(Pose setStart) {
        startPose = setStart;
    }

    public void setPose(Pose setPose) {

        //reset

        odometryFusion.setPosition(MathFunctions.subtractPoses(setPose, startPose));
    }

    public void update() {
        pos = odometryFusion.getPosition();
        vel = odometryFusion.getVelocity();

        totalHeading += MathFunctions.getSmallestAngleDifference(pos.getHeading(AngleUnit.RADIANS), previousHeading);
        previousHeading = pos.getHeading(AngleUnit.RADIANS);
    }

    public double getTotalHeading() {
        return totalHeading;
    }

    public double getForwardMultiplier() {
        return 0;
    }

    public double getLateralMultiplier() {
        return 0;
    }

    public double getTurningMultiplier() {
        return 0;
    }

    public void resetIMU() {

    }
}
