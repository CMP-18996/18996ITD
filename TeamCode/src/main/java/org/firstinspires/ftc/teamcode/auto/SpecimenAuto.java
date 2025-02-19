package org.firstinspires.ftc.teamcode.auto;

/*
@Autonomous(name = "Specimen auto :(")
public class SpecimenAuto extends OpMode {
    private Follower follower;

    //private Localizer localizer;

    private Timer pathTimer;

    private int pathState;

    private int cyclesDone = 0;

    private final Pose startPose = new Pose(7.5625, 55.3125, Math.toRadians(0));

    private final Pose chamberPose = new Pose(38, 66, Math.toRadians(0));

    private final Pose spikePickup1 = new Pose(30.59, 46.81, Math.toRadians(301));

    private final Pose spikePickup2 = new Pose(30.24, 38.80, Math.toRadians(300));

    private final Pose spikePickup3 = new Pose(31.72 + 0.5, 31.29 -1, Math.toRadians(295));

    //private final Pose spikeDrop1 = new Pose(30.59, 46.81, Math.toRadians(240));

    //private final Pose spikeDrop2 = new Pose(30.2, 30.8, Math.toRadians(232));

    //private final Pose spikeDrop3 = new Pose(31.7, 31.2, Math.toRadians(225));

    private final Pose wallPose = new Pose(7.5, 28, 0);

    private final Pose parkPose = new Pose(15, 20, Math.toRadians(45));

    private PathChain scorePreloadedSpecimen, pickupSpike1, pickupSpike2, pickupSpike3, dropSpike1, dropSpike2, dropSpike3, spike3toSpecimen, wallToChamber, chamberToWall, park;

    private Robot robot;

    public void buildPaths() {
        scorePreloadedSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(chamberPose)))
                .setConstantHeadingInterpolation(chamberPose.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        pickupSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(spikePickup1)))
                .setLinearHeadingInterpolation(chamberPose.getHeading(), spikePickup1.getHeading())
                .build();

        pickupSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(spikePickup2)))
                .setLinearHeadingInterpolation(chamberPose.getHeading(), spikePickup2.getHeading())
                .build();

        pickupSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(spikePickup3)))
                .setLinearHeadingInterpolation(chamberPose.getHeading(), spikePickup3.getHeading())
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
                if(follower.getCurrentTValue() > 0.95) {
                    CommandScheduler.getInstance().schedule(new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.OPEN));

                    setPathState(2);
                }
                break;
            case 2:
                // RETRACT AND MOVE TO SPIKE 1
                if(pathTimer.getElapsedTime() > 300) {
                    CommandScheduler.getInstance().schedule(new SpecimenSetArmPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenArmState.WALL));

                    follower.followPath(pickupSpike1,true);
                    setPathState(3);
                }
                break;
            case 3:
                // GET BLOCK
                if(follower.getCurrentTValue() > 0.9) {
                    CommandScheduler.getInstance().schedule(new AutoExtend(robot.extension, robot.intake));

                    setPathState(4);
                }
                break;
            case 4:
                // swing
                if(!robot.intake.getCurrentColor().equals(IntakeSubsystem.Color.NONE) || pathTimer.getElapsedTime() > 1000) {
                    CommandScheduler.getInstance().schedule(new AutoHoldSampleToMove(robot.extension, robot.intake));

                    follower.turnDegrees(80, false);
                    setPathState(5);
                }
                break;
            case 5:
                // throw !!!
                if(pathTimer.getElapsedTime() > 1000) {
                    CommandScheduler.getInstance().schedule(new AutoTossToHumanPlayer(robot.intake));

                    setPathState(6);
                }
                break;
            case 6:
                // RETRACT AND MOVE TO SPIKE 2
                if(pathTimer.getElapsedTime() > 300) {
                    robot.extension.setExtendedEncoderValue(400);

                    follower.followPath(pickupSpike2,true);
                    setPathState(7);
                }
                break;
            case 7:
                // GET BLOCK
                if(follower.getCurrentTValue() > 0.9) {
                    CommandScheduler.getInstance().schedule(new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.PICK_UP));
                    CommandScheduler.getInstance().schedule(new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.PICK_UP));

                    robot.extension.setExtendedEncoderValue(620);

                    setPathState(8);
                }
                break;
            case 8:
                // swing
                if(!robot.intake.getCurrentColor().equals(IntakeSubsystem.Color.NONE) || pathTimer.getElapsedTime() > 1000) {
                    CommandScheduler.getInstance().schedule(new AutoHoldSampleToMove(robot.extension, robot.intake));

                    follower.turnDegrees(80, false);
                    setPathState(9);
                }
                break;
            case 9:
                // throw !!!
                if(pathTimer.getElapsedTime() > 1000) {
                    CommandScheduler.getInstance().schedule(new AutoTossToHumanPlayer(robot.intake));

                    setPathState(10);
                }
                break;
            case 10:
                // RETRACT AND MOVE TO SPIKE 3
                if(pathTimer.getElapsedTime() > 300) {
                    robot.extension.setExtendedEncoderValue(400);

                    follower.followPath(pickupSpike3,true);
                    setPathState(11);
                }
                break;
            case 11:
                // GET BLOCK
                if(follower.getCurrentTValue() > 0.9) {
                    robot.extension.setExtendedEncoderValue(620);
                    CommandScheduler.getInstance().schedule(new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.PICK_UP));
                    CommandScheduler.getInstance().schedule(new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.PICK_UP));

                    setPathState(12);
                }
                break;
            case 12:
                // swing
                if(!robot.intake.getCurrentColor().equals(IntakeSubsystem.Color.NONE) || pathTimer.getElapsedTime() > 1000) {
                    CommandScheduler.getInstance().schedule(new AutoHoldSampleToMove(robot.extension, robot.intake));

                    follower.turnDegrees(80, false);
                    setPathState(13);
                }
                break;
            case 13:
                // throw !!!
                if(pathTimer.getElapsedTime() > 1000) {
                    CommandScheduler.getInstance().schedule(new AutoTossToHumanPlayer(robot.intake));

                    setPathState(14);
                }
                break;
            case 14:
                if(pathTimer.getElapsedTime() > 300) {
                    robot.extension.setExtendedEncoderValue(200);
                    CommandScheduler.getInstance().schedule(new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.REST));
                    CommandScheduler.getInstance().schedule(new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.REST));
                    CommandScheduler.getInstance().schedule(new IntakeSetRollerState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.DISABLED));

                    CommandScheduler.getInstance().schedule(new SpecimenSetArmPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenArmState.WALL));
                    CommandScheduler.getInstance().schedule(new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.OPEN));

                    follower.followPath(spike3toSpecimen);
                    setPathState(15);
                }
                break;
            case 15:
                if(follower.getCurrentTValue() > 0.92) {
                    CommandScheduler.getInstance().schedule(new AutoSpecimenGrab(robot.specimen));

                    setPathState(16);
                }
                break;

            case 16:
                if(pathTimer.getElapsedTime() > 300) {
                    CommandScheduler.getInstance().schedule(new ExtensionSetPosition_INST(robot.extension, ExtensionSubsystem.ExtensionState.TRANSFER));

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
        robot = new Robot(hardwareMap, Team.BLUE, Subsystems.INTAKE, Subsystems.DEPOSIT, Subsystems.EXTENSION, Subsystems.SPECIMEN);
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }
}

 */