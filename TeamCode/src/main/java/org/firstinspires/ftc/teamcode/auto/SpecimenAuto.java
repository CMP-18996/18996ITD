package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetArmPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetGripperPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.Team;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Specimen auto :(", group = "Examples")
public class SpecimenAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(7.5625, 55.3125, Math.toRadians(0));

    /** Scoring Pose of our robot at submersible */
    private final Pose scorePose = new Pose(35, 55.3125, Math.toRadians(0));

    /** Going from scorePose to behind first grounded sample */
    private final Pose pickup1Pose1 = new Pose(24, 24);
    private final Pose pickup1Pose2 = new Pose(72, 43);
    private final Pose pickup1Pose3 = new Pose(64, 28);
    private final Pose pickup1PoseEND = new Pose(65, 22);

    /** Depositing first grounded sample in observation zone */
    private final Pose deposit1Pose = new Pose(15, 27, Math.toRadians(180));

    /** Going from deposit1Pose to behind second grounded sample */
    private final Pose pickup2Pose1 = new Pose(70, 36, Math.toRadians(0));
    private final Pose pickup2Pose2 = new Pose(60, 15, Math.toRadians(0));

    /** Depositing second grounded sample in observation zone */
    private final Pose deposit2Pose = new Pose(15, 15, Math.toRadians(0));

    /** Going from deposit2Pose to behind third grounded sample */
    private final Pose pickup3Pose1 = new Pose(72, 24, Math.toRadians(0));
    private final Pose pickup3Pose2 = new Pose(60, 9, Math.toRadians(0));

    /** Depositing third grounded sample in observation zone */
    private final Pose deposit3Pose = new Pose(15, 9, Math.toRadians(0));

    /** From submersible direct to top of observation zone */
    private final Pose specimenPose = new Pose(15, 24, Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain scorePreloadedSpecimen, pushSamples, pushSample1, pushSample2, pushSample3, scoreGroundSpecimen, scoreGroundSpecimenAlt, returnGroundSpecimen;

    private Robot robot;
    public void buildPaths() {
        /* Goes from startPose to the scorePose for the preloaded specimen dropoff */
        scorePreloadedSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        /*pushSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickup1Pose1), new Point(pickup1Pose2), new Point(pickup1Pose3), new Point(pickup1PoseEND)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addPath(new BezierCurve(new Point(pickup1PoseEND), new Point(deposit1Pose)))
                .setConstantHeadingInterpolation(Math.PI)
                .setReversed(true)
                .build();

        pushSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(deposit1Pose), new Point(pickup2Pose1), new Point(pickup2Pose2)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addPath(new BezierLine(new Point(pickup2Pose2), new Point(deposit2Pose)))
                .setConstantHeadingInterpolation(Math.PI)
                .setReversed(true)
                .build();

        pushSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(deposit2Pose), new Point(pickup3Pose1), new Point(pickup3Pose2)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addPath(new BezierLine(new Point(pickup3Pose2), new Point(deposit3Pose)))
                .setConstantHeadingInterpolation(Math.PI)
                .setReversed(true)
                .build();*/

        pushSamples = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickup1Pose1), new Point(pickup1Pose2), new Point(pickup1Pose3), new Point(pickup1PoseEND)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addPath(new BezierCurve(new Point(pickup1PoseEND), new Point(deposit1Pose)))
                .setConstantHeadingInterpolation(Math.PI)
                .setReversed(true)

                .addPath(new BezierCurve(new Point(deposit1Pose), new Point(pickup2Pose1), new Point(pickup2Pose2)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .setReversed(false)
                .addPath(new BezierLine(new Point(pickup2Pose2), new Point(deposit2Pose)))
                .setConstantHeadingInterpolation(Math.PI)
                .setReversed(true)

                .addPath(new BezierCurve(new Point(deposit2Pose), new Point(pickup3Pose1), new Point(pickup3Pose2)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .setReversed(false)
                .addPath(new BezierLine(new Point(pickup3Pose2), new Point(deposit3Pose)))
                .setConstantHeadingInterpolation(Math.PI)
                .setReversed(true)
                .build();

        scoreGroundSpecimenAlt = follower.pathBuilder()
                .addPath(new BezierLine(new Point(deposit3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(Math.toRadians(64), 0)
                .build();

        scoreGroundSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(Math.toRadians(47), 0)
                .build();

        returnGroundSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(specimenPose)))
                .setLinearHeadingInterpolation(Math.toRadians(47), 0)
                .setReversed(true)
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                CommandScheduler.getInstance().schedule(new SpecimenSetArmPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenArmState.CHAMBER));
                follower.followPath(scorePreloadedSpecimen, true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.OPEN));
                    CommandScheduler.getInstance().schedule(new SpecimenSetArmPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenArmState.WALL));
                    follower.followPath(pushSamples,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.CLOSED));
                    CommandScheduler.getInstance().schedule(new SpecimenSetArmPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenArmState.CHAMBER));
                    follower.followPath(scoreGroundSpecimenAlt,true);
                    setPathState(5);
                }
                break;
            case 6:
            case 8:
            case 10:
                if(!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.CLOSED));
                    CommandScheduler.getInstance().schedule(new SpecimenSetArmPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenArmState.CHAMBER));
                    follower.followPath(scoreGroundSpecimen,true);
                    setPathState(pathState == 10 ? -1434 : pathState + 1);
                }
                break;
            case 5:
            case 7:
            case 9:
                if(!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.OPEN));
                    CommandScheduler.getInstance().schedule(new SpecimenSetArmPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenArmState.WALL));
                    follower.followPath(returnGroundSpecimen,true);
                    setPathState(pathState + 1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        CommandScheduler.getInstance().run();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(1);
        buildPaths();

        CommandScheduler.getInstance().reset();
        //It doesn't matter which team it is due to symmetry :)
        robot = new Robot(hardwareMap, Team.BLUE, Subsystems.SPECIMEN);
        CommandScheduler.getInstance().schedule(new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.CLOSED));
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}

