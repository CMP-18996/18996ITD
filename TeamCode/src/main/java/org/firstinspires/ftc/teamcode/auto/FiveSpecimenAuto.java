package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.commands.complexCommands.ExtendToIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.autoCommands.AutoSpecimenDeposit;
import org.firstinspires.ftc.teamcode.common.commands.autoCommands.AutoSpecimenGrab;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.HoldSampleCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.IdleIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.PickupSampleCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.RestIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakePivotSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeRollerSetState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetArmPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetGripperPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.Color;
import org.firstinspires.ftc.teamcode.common.robot.MatchDataStorage;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "5 Specimen auto :(")
public class FiveSpecimenAuto extends OpMode {
    private Follower follower;

    //private Localizer localizer;

    private Timer pathTimer;

    private int pathState;

    private int cyclesDone = 0;

    private final Pose startPose = new Pose(7.5625, 55.3125, Math.toRadians(0));

    private final Pose preloadDepositPose = new Pose(39, 72, Math.toRadians(0));

    private final Pose chamberPose = new Pose(38, 66, Math.toRadians(0));

    private final Pose spikePickup1 = new Pose(27.5, 46, Math.toRadians(301));

    private final Pose spikePickup2 = new Pose(28.5, 38, Math.toRadians(300));

    private final Pose spikePickup3 = new Pose(31, 30.29, Math.toRadians(295));

    //private final Pose spikeDrop1 = new Pose(30.59, 46.81, Math.toRadians(240));

    //private final Pose spikeDrop2 = new Pose(30.2, 30.8, Math.toRadians(232));

    //private final Pose spikeDrop3 = new Pose(31.7, 31.2, Math.toRadians(225));

    private final Pose wallPose = new Pose(7.5, 28, 0);

    private final Pose parkPose = new Pose(15, 20, Math.toRadians(45));

    private PathChain scorePreloadedSpecimen, pickupSpike1, pickupSpike2, pickupSpike3, dropSpike1, dropSpike2, dropSpike3, spike3toSpecimen, wallToChamber, chamberToWall, park;

    private Robot robot;

    public void buildPaths() {
        scorePreloadedSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadDepositPose)))
                .setConstantHeadingInterpolation(chamberPose.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        pickupSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(spikePickup1)))
                .setLinearHeadingInterpolation(chamberPose.getHeading(), spikePickup1.getHeading())
                .addParametricCallback(0.45, () -> CommandScheduler.getInstance().schedule(new RestIntakeCommand(robot.intake)))
                .addParametricCallback(0.5, () -> CommandScheduler.getInstance().schedule(new ExtendToIntakeCommand(robot.extension, robot.intake)))
                .build();

        pickupSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(spikePickup2)))
                .setLinearHeadingInterpolation(chamberPose.getHeading(), spikePickup2.getHeading())
                .addParametricCallback(0.45, () -> CommandScheduler.getInstance().schedule(new RestIntakeCommand(robot.intake)))
                .build();

        pickupSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(spikePickup3)))
                .setLinearHeadingInterpolation(chamberPose.getHeading(), spikePickup3.getHeading())
                .addParametricCallback(0.45, () -> CommandScheduler.getInstance().schedule(new RestIntakeCommand(robot.intake)))
                .build();

        spike3toSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spikePickup3), new Point(wallPose)))
                .setLinearHeadingInterpolation(spikePickup3.getHeading(), wallPose.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        wallToChamber = follower.pathBuilder()
                .addPath(new BezierLine(new Point(wallPose), new Point(chamberPose)))
                .setConstantHeadingInterpolation(chamberPose.getHeading())
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        chamberToWall = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(wallPose)))
                .setConstantHeadingInterpolation(wallPose.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(parkPose)))
                .setTangentHeadingInterpolation()
                .setZeroPowerAccelerationMultiplier(4)
                .addParametricCallback(0.5, () -> CommandScheduler.getInstance().schedule(new ExtendToIntakeCommand(robot.extension, robot.intake)))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            // RAISE STUFF
            case 0:
                CommandScheduler.getInstance().schedule(new SpecimenSetArmPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenArmState.CHAMBER));

                follower.followPath(scorePreloadedSpecimen, true);
                setPathState(1);
                break;
            case 1:
                // LET GO !!!!
                if(follower.getCurrentTValue() > 0.97) {
                    CommandScheduler.getInstance().schedule(new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.OPEN));

                    setPathState(2);
                }
                break;
            case 2:
                // RETRACT AND MOVE TO SPIKE 1
                if(pathTimer.getElapsedTime() > 300) {
                    CommandScheduler.getInstance().schedule(new SpecimenSetArmPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenArmState.WALL));

                    robot.intake.setIntakeLockAngle(spikePickup1.getHeading());
                    robot.intake.setIntakePivotState(IntakeSubsystem.IntakePivotState.PIVOT_LOCK);

                    follower.followPath(pickupSpike1,true);
                    setPathState(3);
                }
                break;
            case 3:
                // GET BLOCK
                if(follower.getCurrentTValue() > 0.975) {
                    CommandScheduler.getInstance().schedule(new PickupSampleCommand(robot.intake));

                    setPathState(4);
                }
                break;
            case 4:
                // swing
                if(!robot.intake.getCurrentColor().equals(Color.NONE) || pathTimer.getElapsedTime() > 1000) {
                    CommandScheduler.getInstance().schedule(new HoldSampleCommand(robot.intake));
                    CommandScheduler.getInstance().schedule(new ExtensionSetPosition_INST(robot.extension, ExtensionSubsystem.ExtensionState.INSPECTION));

                    follower.turnDegrees(80, false);
                    setPathState(5);
                }
                break;
            case 5:
                // throw !!!
                if(pathTimer.getElapsedTime() > 750) {
                    robot.intake.setIntakeRollerState(IntakeSubsystem.IntakeRollerState.REVERSING);
                    CommandScheduler.getInstance().schedule(new ExtensionSetPosition_INST(robot.extension, ExtensionSubsystem.ExtensionState.EXTENDED));

                    setPathState(6);
                }
                break;
            case 6:
                // RETRACT AND MOVE TO SPIKE 2
                if(pathTimer.getElapsedTime() > 300) {
                    CommandScheduler.getInstance().schedule(new IdleIntakeCommand(robot.intake));
                    robot.intake.setIntakeLockAngle(spikePickup2.getHeading());
                    robot.intake.setIntakePivotState(IntakeSubsystem.IntakePivotState.PIVOT_LOCK);

                    follower.followPath(pickupSpike2,true);
                    setPathState(7);
                }
                break;
            case 7:
                // GET BLOCK
                if(!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(new PickupSampleCommand(robot.intake));

                    setPathState(8);
                }
                break;
            case 8:
                // swing
                if(!robot.intake.getCurrentColor().equals(Color.NONE) || pathTimer.getElapsedTime() > 1000) {
                    CommandScheduler.getInstance().schedule(new HoldSampleCommand(robot.intake));
                    CommandScheduler.getInstance().schedule(new ExtensionSetPosition_INST(robot.extension, ExtensionSubsystem.ExtensionState.INSPECTION));


                    follower.turnDegrees(80, false);
                    setPathState(9);
                }
                break;
            case 9:
                // throw !!!
                if(pathTimer.getElapsedTime() > 1000) {
                    robot.intake.setIntakeRollerState(IntakeSubsystem.IntakeRollerState.REVERSING);
                    CommandScheduler.getInstance().schedule(new ExtensionSetPosition_INST(robot.extension, ExtensionSubsystem.ExtensionState.EXTENDED));


                    setPathState(10);
                }
                break;
            case 10:
                // RETRACT AND MOVE TO SPIKE 3
                if(pathTimer.getElapsedTime() > 300) {
                    CommandScheduler.getInstance().schedule(new IdleIntakeCommand(robot.intake));
                    robot.intake.setIntakeLockAngle(spikePickup3.getHeading());
                    robot.intake.setIntakePivotState(IntakeSubsystem.IntakePivotState.PIVOT_LOCK);

                    follower.followPath(pickupSpike3,true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(new PickupSampleCommand(robot.intake));

                    setPathState(12);
                }
                break;
            case 12:
                // swing
                if(!robot.intake.getCurrentColor().equals(Color.NONE) || pathTimer.getElapsedTime() > 1000) {
                    CommandScheduler.getInstance().schedule(new ExtensionSetPosition_INST(robot.extension, ExtensionSubsystem.ExtensionState.MIDDLE));
                    CommandScheduler.getInstance().schedule(new HoldSampleCommand(robot.intake));

                    follower.turnDegrees(80, false);
                    setPathState(13);
                }
                break;
            case 13:
                // throw !!!
                if(pathTimer.getElapsedTime() > 1500) {
                    robot.intake.setIntakeRollerState(IntakeSubsystem.IntakeRollerState.REVERSING);
                    CommandScheduler.getInstance().schedule(new ExtensionSetPosition_INST(robot.extension, ExtensionSubsystem.ExtensionState.EXTENDED));


                    setPathState(14);
                }
                break;
            case 14:
                if(pathTimer.getElapsedTime() > 300) {
                    CommandScheduler.getInstance().schedule(new IdleIntakeCommand(robot.intake));
                    robot.extension.setExtensionState(ExtensionSubsystem.ExtensionState.TRANSFER);

                    follower.followPath(spike3toSpecimen);
                    setPathState(15);
                }
                break;
            case 15:
                if(follower.getCurrentTValue() > 0.92) {
                    CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                            new AutoSpecimenGrab(robot.specimen),
                            new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.TRANSFER),
                            new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.TRANSFER),
                            new IntakePivotSetPosition_INST(robot.intake, IntakeSubsystem.IntakePivotState.PIVOT_TRANSFER),

                            new WaitCommand(500),

                            new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.DISABLED),
                            new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.BUCKET)
                    ));

                    setPathState(16);
                }
                break;

            case 16:
                if(pathTimer.getElapsedTime() > 300) {

                    follower.followPath(wallToChamber);
                    setPathState(17);
                }
                break;

            case 17:
                if(follower.getCurrentTValue() > 0.97) {
                    CommandScheduler.getInstance().schedule(new AutoSpecimenDeposit(robot.specimen));
                    cyclesDone += 1;

                    if(cyclesDone < 4) {
                        follower.followPath(chamberToWall);
                        setPathState(15);
                    }
                    else {
                        follower.followPath(park);
                        setPathState(18);
                    }
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
        follower.update();
        autonomousPathUpdate();
        CommandScheduler.getInstance().run();

        MatchDataStorage.robotPose = follower.getPose();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("FOLLOWER", follower.isBusy());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);

        //localizer = new STATICLocalizer(hardwareMap);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        CommandScheduler.getInstance().reset();

        //It doesn't matter which team it is due to symmetry :)
        robot = new Robot(hardwareMap, Subsystems.INTAKE, Subsystems.DEPOSIT, Subsystems.EXTENSION, Subsystems.SPECIMEN, Subsystems.LIFT);

        robot.intake.setIntakeWristState(IntakeSubsystem.IntakeWristState.TRANSFER);
        robot.intake.setIntakeArmState(IntakeSubsystem.IntakeArmState.TRANSFER);
        robot.intake.setIntakePivotState(IntakeSubsystem.IntakePivotState.PIVOT_TRANSFER);

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.TRANSFER),
                new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.TRANSFER),
                new IntakePivotSetPosition_INST(robot.intake, IntakeSubsystem.IntakePivotState.PIVOT_TRANSFER),

                new WaitCommand(500),

                new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.DISABLED),
                new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.BUCKET)
        ));
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }
}