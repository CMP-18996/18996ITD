package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.odo.STATICLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "6 Sample Park Auto")
public class BucketAuto extends OpMode {

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    private Follower follower;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    private final Pose startPose = new Pose(9, 108, Math.toRadians(270));

    private final Pose depositPose = new Pose(14, 130, Math.toRadians(315));

    private final Pose spikePickup1 = new Pose(24, 124, Math.toRadians(350));

    private final Pose spikePickup2 = new Pose(24, 132, Math.toRadians(270));

    private final Pose spikePickup3 = new Pose(24, 132, Math.toRadians(25));

    private final Pose submersiblePickup = new Pose(60, 96, Math.toRadians(270));
    private final Pose submersibleControl1 = new Pose(41, 123, Math.toRadians(270));
    private final Pose submersibleControl2 = new Pose(62, 115, Math.toRadians(270));

    private final Pose parkPose = new Pose(60, 95, Math.toRadians(270));

    private PathChain scorePreload, spike1, deposit1, spike2, deposit2, spike3, deposit3, sub1, deposit4, sub2, deposit5, park;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(depositPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), depositPose.getHeading())
                .build();

        spike1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(depositPose), new Point(spikePickup1)))
                .setLinearHeadingInterpolation(depositPose.getHeading(), spikePickup1.getHeading())
                .build();

        deposit1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spikePickup1), new Point(depositPose)))
                .setLinearHeadingInterpolation(spikePickup1.getHeading(), depositPose.getHeading())
                .build();

        spike2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(depositPose), new Point(spikePickup2)))
                .setLinearHeadingInterpolation(depositPose.getHeading(), spikePickup2.getHeading())
                .build();

        deposit2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spikePickup2), new Point(depositPose)))
                .setLinearHeadingInterpolation(spikePickup2.getHeading(), depositPose.getHeading())
                .build();

        spike3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(depositPose), new Point(spikePickup3)))
                .setLinearHeadingInterpolation(depositPose.getHeading(), spikePickup3.getHeading())
                .build();

        deposit3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spikePickup3), new Point(depositPose)))
                .setLinearHeadingInterpolation(spikePickup3.getHeading(), depositPose.getHeading())
                .build();

        sub1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(depositPose),
                        new Point(submersibleControl1),
                        new Point(submersibleControl2),
                        new Point(submersiblePickup)
                ))
                .setLinearHeadingInterpolation(depositPose.getHeading(), submersiblePickup.getHeading())
                .build();

        deposit4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(submersiblePickup),
                        new Point(submersibleControl2),
                        new Point(submersibleControl1),
                        new Point(depositPose)
                ))
                .setLinearHeadingInterpolation(submersiblePickup.getHeading(), depositPose.getHeading())
                .build();

        sub2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(depositPose),
                        new Point(submersibleControl1),
                        new Point(submersibleControl2),
                        new Point(submersiblePickup)
                ))
                .setLinearHeadingInterpolation(depositPose.getHeading(), submersiblePickup.getHeading())
                .build();

        deposit5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(submersiblePickup),
                        new Point(submersibleControl2),
                        new Point(submersibleControl1),
                        new Point(depositPose)
                ))
                .setLinearHeadingInterpolation(submersiblePickup.getHeading(), depositPose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(depositPose),
                        new Point(submersibleControl1),
                        new Point(submersibleControl2),
                        new Point(parkPose)
                ))
                .setLinearHeadingInterpolation(depositPose.getHeading(), parkPose.getHeading())
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */
                follower.followPath(spike1,true);
                setPathState(2);
                break;
            case 2:
                follower.followPath(deposit1,true);
                setPathState(3);
                break;
            case 3:
                follower.followPath(spike2,true);
                setPathState(4);
                break;
            case 4:
                follower.followPath(deposit2,true);
                setPathState(5);
                break;
            case 5:
                follower.followPath(spike3,true);
                setPathState(6);
                break;
            case 6:
                follower.followPath(deposit3, true);
                setPathState(7);
                break;
            case 7:
                follower.followPath(sub1,true);
                setPathState(8);
                break;
            case 8:
                follower.followPath(deposit4,true);
                setPathState(9);
                break;
            case 9:
                follower.followPath(sub2,true);
                setPathState(10);
                break;
            case 10:
                follower.followPath(deposit5,true);
                setPathState(11);
                break;
            case 11:
                follower.followPath(park,true);
                setPathState(12);
                break;
            case 12:
                setPathState(-1);
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        // These loop the movements of the robot
        follower.update();

        if (currentGamepad1.cross && !previousGamepad1.cross) {
            autonomousPathUpdate();
        }

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, new STATICLocalizer());
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        setPathState(0);
    }
}
