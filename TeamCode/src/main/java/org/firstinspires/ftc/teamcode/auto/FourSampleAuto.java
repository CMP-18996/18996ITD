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

import org.firstinspires.ftc.teamcode.common.commands.complexCommands.ExtendToIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.PickupSampleCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.common.robot.Color;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.Team;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "4 Sample auto :)")
public class FourSampleAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    private final Pose depositPose = new Pose(12.7, 133.2, Math.toRadians(315));

    private final Pose spikePickup1 = new Pose(17.7, 128.5, Math.toRadians(346));

    private final Pose spikePickup2 = new Pose(20.226, 131.17, Math.toRadians(0));

    private final Pose spikePickup3 = new Pose(18.38, 129.7575, Math.toRadians(24));

    private final Pose submersible = new Pose(72, 98, Math.toRadians(270));

    private PathChain scorePreloadedSample, pickupSpike1, pickupSpike2, pickupSpike3, scoreSpike1, scoreSpike2, scoreSpike3, park;

    private Robot robot;

    public void buildPaths() {
        scorePreloadedSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(depositPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), depositPose.getHeading())
                .build();

        pickupSpike1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(depositPose), new Point(spikePickup1)))
                .setLinearHeadingInterpolation(depositPose.getHeading(), spikePickup1.getHeading())
                .build();

        pickupSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(depositPose), new Point(spikePickup2)))
                .setLinearHeadingInterpolation(depositPose.getHeading(), spikePickup2.getHeading())
                .build();

        pickupSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(depositPose), new Point(spikePickup3)))
                .setLinearHeadingInterpolation(depositPose.getHeading(), spikePickup3.getHeading())
                .build();

        scoreSpike1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(spikePickup1), new Point(depositPose)))
                .setLinearHeadingInterpolation(spikePickup1.getHeading(), depositPose.getHeading())
                .build();

        scoreSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(spikePickup2), new Point(depositPose)))
                .setLinearHeadingInterpolation(spikePickup2.getHeading(), depositPose.getHeading())
                .build();

        scoreSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(spikePickup3), new Point(depositPose)))
                .setLinearHeadingInterpolation(spikePickup3.getHeading(), depositPose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(depositPose), new Point(submersible)))
                .setTangentHeadingInterpolation()
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.lift.setLiftState(LiftSubsystem.LiftState.HIGH_BUCKET);
                robot.deposit.setBucketState(DepositSubsystem.BucketState.DEPOSIT);

                follower.followPath(scorePreloadedSample, true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    robot.deposit.setDepositTrapdoorState(DepositSubsystem.DepositTrapdoorState.OPEN);
                    CommandScheduler.getInstance().schedule(new ExtendToIntakeCommand(robot.extension, robot.intake));
                    robot.intake.setIntakeLockAngle(spikePickup1.getHeading());
                    robot.intake.setIntakePivotState(IntakeSubsystem.IntakePivotState.PIVOT_LOCK);

                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(pickupSpike1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(new PickupSampleCommand(robot.intake));
                    setPathState(4);
                }
                break;
            case 4:
                if (robot.intake.getCurrentColor().equals(Color.YELLOW) || pathTimer.getElapsedTime() > 2000) {
                    CommandScheduler.getInstance().schedule(new TransferSampleCommand(robot.extension, robot.intake, robot.deposit, robot.lift));
                    follower.followPath(scoreSpike1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    robot.deposit.setDepositTrapdoorState(DepositSubsystem.DepositTrapdoorState.OPEN);
                    CommandScheduler.getInstance().schedule(new ExtendToIntakeCommand(robot.extension, robot.intake));
                    robot.intake.setIntakeLockAngle(spikePickup2.getHeading());
                    robot.intake.setIntakePivotState(IntakeSubsystem.IntakePivotState.PIVOT_LOCK);

                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(pickupSpike2,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(new PickupSampleCommand(robot.intake));
                    setPathState(8);
                }
                break;
            case 8:
                if (robot.intake.getCurrentColor().equals(Color.YELLOW) || pathTimer.getElapsedTime() > 2000) {
                    CommandScheduler.getInstance().schedule(new TransferSampleCommand(robot.extension, robot.intake, robot.deposit, robot.lift));
                    follower.followPath(scoreSpike2, true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    robot.deposit.setDepositTrapdoorState(DepositSubsystem.DepositTrapdoorState.OPEN);
                    CommandScheduler.getInstance().schedule(new ExtendToIntakeCommand(robot.extension, robot.intake));
                    robot.intake.setIntakeLockAngle(spikePickup3.getHeading());
                    robot.intake.setIntakePivotState(IntakeSubsystem.IntakePivotState.PIVOT_LOCK);

                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(pickupSpike3,true);
                    setPathState(11);
                }
                break;
            case 11:
                if (robot.intake.getCurrentColor().equals(Color.YELLOW) || pathTimer.getElapsedTime() > 2000) {
                    CommandScheduler.getInstance().schedule(new TransferSampleCommand(robot.extension, robot.intake, robot.deposit, robot.lift));
                    follower.followPath(scoreSpike3, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    robot.deposit.setDepositTrapdoorState(DepositSubsystem.DepositTrapdoorState.OPEN);
                    CommandScheduler.getInstance().schedule(new ExtendToIntakeCommand(robot.extension, robot.intake));
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(park,true);
                    //CommandScheduler.getInstance().schedule(new AutoAscentCommand(robot.extension, robot.intake, robot.lift, robot.deposit));
                    setPathState(13);
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
        telemetry.addData("COLOR", robot.intake.getCurrentColor());
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
        robot = new Robot(hardwareMap, Subsystems.INTAKE, Subsystems.DEPOSIT, Subsystems.LIFT, Subsystems.EXTENSION);
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }
}