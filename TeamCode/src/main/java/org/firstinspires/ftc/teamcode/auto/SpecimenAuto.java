package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.commands.autoCommands.AutoExtend;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.CompactForHangCommand;
import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetMotorState_INST;
import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetArmPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetGripperPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.Team;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Specimen auto :(")
public class SpecimenAuto extends OpMode {
    private Follower follower;

    private Timer pathTimer;

    private int pathState;

    private final Pose startPose = new Pose(7.5625, 55.3125, Math.toRadians(0));

    private final Pose chamberPose = new Pose(33, 55.3125, Math.toRadians(0));

    private final Pose spikePickup1 = new Pose(30.59, 46.81, Math.toRadians(301));

    private final Pose spikePickup2 = new Pose(30.24, 38.80, Math.toRadians(300));

    private final Pose spikePickup3 = new Pose(31.72, 31.29, Math.toRadians(295));

    private final Pose spikeDrop1 = new Pose(30.59, 46.81, Math.toRadians(240));

    private final Pose spikeDrop2 = new Pose(30.24, 30.80, Math.toRadians(232));

    private final Pose spikeDrop3 = new Pose(31.72, 31.29, Math.toRadians(225));

    private final Pose wallPose = new Pose(9.14, 30.13, 0);

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain scorePreloadedSpecimen, pickupSpike1, pickupSpike2, pickupSpike3, dropSpike1, dropSpike2, dropSpike3;

    private Robot robot;

    public void buildPaths() {
        scorePreloadedSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(chamberPose)))
                .setConstantHeadingInterpolation(chamberPose.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        pickupSpike1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(chamberPose), new Point(spikePickup1)))
                .setLinearHeadingInterpolation(chamberPose.getHeading(), spikePickup1.getHeading())
                .build();

        pickupSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(chamberPose), new Point(spikePickup2)))
                .setLinearHeadingInterpolation(chamberPose.getHeading(), spikePickup2.getHeading())
                .build();

        pickupSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(chamberPose), new Point(spikePickup3)))
                .setLinearHeadingInterpolation(chamberPose.getHeading(), spikePickup3.getHeading())
                .build();

        dropSpike1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(spikePickup1), new Point(spikeDrop1)))
                .setLinearHeadingInterpolation(spikePickup1.getHeading(), spikeDrop1.getHeading())
                .build();

        dropSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(spikePickup2), new Point(spikeDrop2)))
                .setLinearHeadingInterpolation(spikePickup2.getHeading(), spikeDrop2.getHeading())
                .build();

        dropSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(spikePickup3), new Point(spikeDrop3)))
                .setLinearHeadingInterpolation(spikePickup3.getHeading(), spikeDrop3.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // RAISE STUFF
                CommandScheduler.getInstance().schedule(new SpecimenSetArmPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenArmState.CHAMBER));

                follower.followPath(scorePreloadedSpecimen, true);
                setPathState(1);
                break;
            case 1:
                // LET GO !!!!
                if(!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.OPEN));

                    setPathState(2);
                }
                break;
            case 2:
                // RETRACT AND MOVE TO SPIKE 1
                if(pathTimer.getElapsedTime() > 1000) {
                    CommandScheduler.getInstance().schedule(new SpecimenSetArmPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenArmState.WALL));

                    follower.followPath(pickupSpike1,true);
                    setPathState(3);
                }
                break;
            case 3:
                // GET BLOCK
                if(!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(new AutoExtend(robot.extension, robot.intake, robot.lift));

                    setPathState(4);
                }
                break;
            case 4:
                // swing
                //if(!robot.intake.getCurrentColor().equals(IntakeSubsystem.Color.NONE)) {
                    if(pathTimer.getElapsedTime() > 1000) {
                    CommandScheduler.getInstance().schedule(new IntakeSetMotorState_INST(robot.intake, IntakeSubsystem.IntakeMotorState.HOLD));

                    follower.followPath(dropSpike1);
                    setPathState(5);
                }
                break;
            case 5:
                // throw !!!
                if(!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(new IntakeSetMotorState_INST(robot.intake, IntakeSubsystem.IntakeMotorState.REVERSING));

                    setPathState(6);
                }
                break;
            case 6:
                // RETRACT AND MOVE TO SPIKE 2
                if(pathTimer.getElapsedTime() > 1000) {
                    CommandScheduler.getInstance().schedule(new IntakeSetMotorState_INST(robot.intake, IntakeSubsystem.IntakeMotorState.DISABLED));
                    CommandScheduler.getInstance().schedule(new ExtensionSetPosition_INST(robot.extension, ExtensionSubsystem.ExtensionState.TRANSFER));

                    follower.followPath(pickupSpike2,true);
                    setPathState(7);
                }
                break;
            case 7:
                // GET BLOCK
                if(!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(new AutoExtend(robot.extension, robot.intake, robot.lift));

                    setPathState(8);
                }
                break;
            case 8:
                // swing
                //if(!robot.intake.getCurrentColor().equals(IntakeSubsystem.Color.NONE)) {
                if(pathTimer.getElapsedTime() > 1000) {
                    CommandScheduler.getInstance().schedule(new IntakeSetMotorState_INST(robot.intake, IntakeSubsystem.IntakeMotorState.HOLD));

                    follower.followPath(dropSpike2);
                    setPathState(9);
                }
                break;
            case 9:
                // throw !!!
                if(!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(new IntakeSetMotorState_INST(robot.intake, IntakeSubsystem.IntakeMotorState.REVERSING));

                    setPathState(10);
                }
                break;
            case 10:
                // RETRACT AND MOVE TO SPIKE 3
                if(pathTimer.getElapsedTime() > 1000) {
                    CommandScheduler.getInstance().schedule(new IntakeSetMotorState_INST(robot.intake, IntakeSubsystem.IntakeMotorState.DISABLED));
                    CommandScheduler.getInstance().schedule(new ExtensionSetPosition_INST(robot.extension, ExtensionSubsystem.ExtensionState.TRANSFER));

                    follower.followPath(pickupSpike3,true);
                    setPathState(11);
                }
                break;
            case 11:
                // GET BLOCK
                if(!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(new AutoExtend(robot.extension, robot.intake, robot.lift));

                    setPathState(12);
                }
                break;
            case 12:
                // swing
                //if(!robot.intake.getCurrentColor().equals(IntakeSubsystem.Color.NONE)) {
                if(pathTimer.getElapsedTime() > 1000) {
                    CommandScheduler.getInstance().schedule(new IntakeSetMotorState_INST(robot.intake, IntakeSubsystem.IntakeMotorState.HOLD));

                    follower.followPath(dropSpike3);
                    setPathState(13);
                }
                break;
            case 13:
                // throw !!!
                if(!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(new IntakeSetMotorState_INST(robot.intake, IntakeSubsystem.IntakeMotorState.REVERSING));

                    setPathState(14);
                }
                break;
            case 14:
                if(pathTimer.getElapsedTime() > 1000) {
                    CommandScheduler.getInstance().schedule(new CompactForHangCommand(robot.extension, robot.intake, robot.deposit, robot.lift, robot.specimen));

                    setPathState(15);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

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

    @Override
    public void init() {
        pathTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        CommandScheduler.getInstance().reset();

        //It doesn't matter which team it is due to symmetry :)
        robot = new Robot(hardwareMap, Team.BLUE, Subsystems.INTAKE, Subsystems.DEPOSIT, Subsystems.EXTENSION, Subsystems.SPECIMEN);
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }
}