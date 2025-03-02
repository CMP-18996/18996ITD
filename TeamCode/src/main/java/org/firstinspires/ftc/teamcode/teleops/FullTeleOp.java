package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.autoCommands.AutoSpecimenDeposit;
import org.firstinspires.ftc.teamcode.common.commands.autoCommands.AutoSpecimenGrab;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.HoldSampleCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.IdleIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.PickupSampleCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositTrapdoorPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.hang.HangCommand_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakePivotSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeRollerSetState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetColorSensorStatus_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetArmPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetGripperPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.Color;
import org.firstinspires.ftc.teamcode.common.robot.MatchDataStorage;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.List;

@TeleOp
public class FullTeleOp extends CommandOpMode {
    private Robot robot;
    private int pathState = 0;
    private Timer pathTimer;
    private Color specimenColor = Color.BLUE;
    private boolean transfer = false;
    //private boolean transferring = false;

    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;

    private final Pose wallPose = new Pose(7.5, 28, Math.toRadians(0));

    private final Pose chamberPose = new Pose(38, 68, Math.toRadians(0));

    private final Pose specControl1 = new Pose(23.21, 73.87);
    private final Pose specControl2 = new Pose(24.46, 42.18);

    private final Pose humanPlayerDepositPose = new Pose(48, 20);

    private double previousPower = 0;

    private Follower follower;

    private PathChain wallToChamber, chamberToWall;

    public void runOpMode() throws InterruptedException {
        waitForStart();
        reset();
        initialize();

        gamepad1.rumbleBlips(1);

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
    }

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, Subsystems.ALL);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }

        robot.specimen.setSpecimenArmState(SpecimenSubsystem.SpecimenArmState.WALL);
        CommandScheduler.getInstance().schedule(new IdleIntakeCommand(robot.intake));

        pathTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(MatchDataStorage.robotPose);

        follower.startTeleopDrive();

        wallToChamber = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(wallPose), new Point(specControl1), new Point(specControl2), new Point(chamberPose)))
                .setConstantHeadingInterpolation(chamberPose.getHeading())
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        chamberToWall = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(wallPose)))
                .setConstantHeadingInterpolation(wallPose.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);

        // MAIN DRIVER
        // MAIN DRIVER
        // MAIN DRIVER

        // Lift High Bucket
        gamepad_1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                /*
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new DepositTrapdoorPosition_INST(robot.deposit, DepositSubsystem.DepositTrapdoorState.CLOSED),
                                //new ZeroLift(robot.lift),
                                new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.HIGH_BUCKET)
                        ),
                        new ScheduleCommand(
                                new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.TRANSFER),
                                new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.TRANSFER)
                        ),
                        () -> !robot.lift.getLiftState().equals(LiftSubsystem.LiftState.HIGH_BUCKET)
                )
                */
                () -> robot.lift.setLiftState(LiftSubsystem.LiftState.HIGH_BUCKET)
        );

        // MANUAL HP DEPOSIT
        gamepad_1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                        new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.EJECT),
                        new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.EJECT),

                        new WaitCommand(200),

                        new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.REVERSING),

                        new WaitCommand(1000),

                        new IdleIntakeCommand(robot.intake)
                )
        );

        gamepad_1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new SequentialCommandGroup (
                        new TransferSampleCommand(robot.extension, robot.intake, robot.deposit, robot.lift),
                        new WaitCommand(100),
                        new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.HIGH_BUCKET)
                        //new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.DEPOSIT)
                )
        );

        // deposit deposit
        gamepad_1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new SequentialCommandGroup(
                        new DepositTrapdoorPosition_INST(robot.deposit, DepositSubsystem.DepositTrapdoorState.CLOSED),
                        new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.DEPOSIT),
                        new WaitCommand(500),
                        new DepositTrapdoorPosition_INST(robot.deposit, DepositSubsystem.DepositTrapdoorState.OPEN),
                        new WaitCommand(500),
                        new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.TRANSFER),
                        new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.TRANSFER)
                )
        );

        // AUTO SPECIMEN CYCLE
        gamepad_1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> {
                    follower.setPose(new Pose(7.5, 28, 0));
                    follower.followPath(chamberToWall);
                    setPathState(1);
                }
        );

        gamepad_1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                () -> {
                    follower.startTeleopDrive();
                    robot.extension.setExtensionState(ExtensionSubsystem.ExtensionState.CUSTOM);
                    setPathState(0);
                }
        );

        gamepad_1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                () -> {
                    // compact for hang?
                }
        );

        // Specimen Claw
        gamepad_1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ConditionalCommand(
                        new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.CLOSED),
                        new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.OPEN),
                        () -> robot.specimen.getSpecimenGripperState().equals(SpecimenSubsystem.SpecimenGripperState.OPEN)
                )
        );

        // Intake el block 🥸🥸 !!!!!!
        gamepad_1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new ScheduleCommand(new PickupSampleCommand(robot.intake))
        ).whenReleased(
                () -> {
                    if(robot.intake.getCurrentColor().equals(Color.NONE)) {
                        schedule(new IdleIntakeCommand(robot.intake));
                    }
                }
        );

        // fuck ftclib
        // INTAKE PIVOT, deposit sample, deposit hp
        gamepad_1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                () -> {
                    if(robot.intake.getIntakeWristState().equals(IntakeSubsystem.IntakeWristState.EJECT)) {
                        schedule(
                                new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.REVERSING)
                        );
                    }
                    else if(!robot.lift.getLiftState().equals(LiftSubsystem.LiftState.TRANSFER)) {
                        schedule(
                               //new DepositTrapdoorPosition_INST(robot.deposit, DepositSubsystem.DepositTrapdoorState.OPEN)
                                new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.DEPOSIT)
                        );
                    }
                    else {
                        schedule(
                                new ConditionalCommand(
                                        new IntakePivotSetPosition_INST(robot.intake, IntakeSubsystem.IntakePivotState.PIVOT_90),
                                        new IntakePivotSetPosition_INST(robot.intake, IntakeSubsystem.IntakePivotState.PIVOT_0),
                                        () -> !robot.intake.getIntakePivotState().equals(IntakeSubsystem.IntakePivotState.PIVOT_90)
                                )
                        );
                    }
                }
        ).whenReleased(
                () -> {
                    if(robot.intake.getIntakeWristState().equals(IntakeSubsystem.IntakeWristState.EJECT)) {
                        schedule(new IdleIntakeCommand(robot.intake));
                    }
                    else if(!robot.lift.getLiftState().equals(LiftSubsystem.LiftState.TRANSFER)) {
                        schedule(
                                new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.TRANSFER),
                                new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.TRANSFER)
                        );
                    }
                    else {}
                }
        );

        // Auto hp deposit
        gamepad_1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                () -> {
                    gamepad1.rumbleBlips(1);
                    robot.extension.setExtensionState(ExtensionSubsystem.ExtensionState.TRANSFER);
                    robot.intake.setIntakeArmState(IntakeSubsystem.IntakeArmState.EJECT);
                    robot.intake.setIntakeWristState(IntakeSubsystem.IntakeWristState.EJECT);
                    robot.intake.setIntakePivotState(IntakeSubsystem.IntakePivotState.PIVOT_0);

                    follower.followPath(
                            follower.pathBuilder()
                                    .addPath(new BezierLine(follower.getPose(),
                                            new Pose(follower.getPose().getX(), follower.getPose().getY() - 15)))
                                    .setConstantHeadingInterpolation(90)
                                    .setZeroPowerAccelerationMultiplier(10)
                                    .build()
                    );
                    setPathState(4);
                }
        );

        // FLOOR GRAB
        gamepad_1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new ScheduleCommand(
                        new IntakePivotSetPosition_INST(robot.intake, IntakeSubsystem.IntakePivotState.PIVOT_0),
                        new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.ACTIVE),
                        new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.FLOOR),
                        new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.FLOOR)
                )
        ).whenReleased(
                () -> {
                    if(robot.intake.getCurrentColor().equals(Color.NONE)) {
                        schedule(new IdleIntakeCommand(robot.intake));
                    }
                }
        );

        //  SECOND DRIVER
        //  SECOND DRIVER
        //  SECOND DRIVER
        gamepad_2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new ConditionalCommand(
                        new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.CLOSED),
                        new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.OPEN),
                        () -> robot.specimen.getSpecimenGripperState().equals(SpecimenSubsystem.SpecimenGripperState.OPEN)
                )
        );

        // Specimen Arm to Chamber
        gamepad_2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SpecimenSetArmPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenArmState.CHAMBER)
        );

        // Specimen Arm to Wall
        gamepad_2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SpecimenSetArmPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenArmState.WALL)
        );

        // Enable/Disable Color Sensor
        gamepad_2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ConditionalCommand(
                        new IntakeSetColorSensorStatus_INST(robot.intake, IntakeSubsystem.ColorSensorStatus.DISABLED),
                        new IntakeSetColorSensorStatus_INST(robot.intake, IntakeSubsystem.ColorSensorStatus.ENABLED),
                        () -> robot.intake.getColorSensorStatus().equals(IntakeSubsystem.ColorSensorStatus.ENABLED)
                )
        );

        gamepad_2.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new HangCommand_INST(robot.hang, HangSubsystem.HangState.UP)
        ).whenReleased(
                new HangCommand_INST(robot.hang, HangSubsystem.HangState.OFF)
        );

        gamepad_2.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new HangCommand_INST(robot.hang, HangSubsystem.HangState.DOWN)
        ).whenReleased(
                new HangCommand_INST(robot.hang, HangSubsystem.HangState.HOLD)
        );

        pathTimer.resetTimer();
    }

    @Override
    public void run() {
        telemetry.addData("LIFT STATE !", robot.lift.getLiftState());

        CommandScheduler.getInstance().run();

        telemetry.addData("LIFT STATE 2", robot.lift.getLiftState());


        // Extension Triggers
        double power =  Math.pow(gamepad_1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), 3);

        if(power != previousPower) {
            robot.extension.setExtensionMotorPower(power);
            previousPower = power;
        }

        if(!robot.intake.getCurrentColor().equals(Color.NONE) &&
            !robot.intake.getCurrentColor().equals(robot.intake.getPreviousColor())) {

            gamepad1.rumbleBlips(2);

            if (robot.intake.getCurrentColor().equals(Color.YELLOW)) { // yellow!
                if (transfer) {
                    schedule(new SequentialCommandGroup (
                            new TransferSampleCommand(robot.extension, robot.intake, robot.deposit, robot.lift),
                            new WaitCommand(100),
                            new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.HIGH_BUCKET)
                            //new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.DEPOSIT)
                    ));
                } else {
                    schedule(new HoldSampleCommand(robot.intake));
                }
            } else if (robot.intake.getCurrentColor().equals(specimenColor)) { // red or blue, the good one
                if (transfer) {
                    schedule(
                            new SequentialCommandGroup (
                                    new TransferSampleCommand(robot.extension, robot.intake, robot.deposit, robot.lift),
                                    new WaitCommand(100),
                                    new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.HIGH_BUCKET),
                                    new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.DEPOSIT)
                            )
                    );
                } else {
                    schedule(new HoldSampleCommand(robot.intake));
                }
            } else { // bad
                schedule(
                        new SequentialCommandGroup(
                                new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.EJECT),
                                new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.REVERSING),

                                new WaitCommand(100),

                                new IdleIntakeCommand(robot.intake)
                        )
                );
            }
        }

        telemetry.addData("PATH STATE", pathState);
        telemetry.addData("LIFT STATE", robot.lift.getLiftState());
        telemetry.addData("X VEL", follower.poseUpdater.getVelocity().getXComponent());
        telemetry.addData("Y VEL", follower.poseUpdater.getVelocity().getYComponent());
        telemetry.addData("H VEL", follower.poseUpdater.getAngularVelocity());

        telemetry.addData("Lift error", robot.lift.getError());
        telemetry.addData("Lift power", robot.lift.getPower());
        telemetry.update();

        if(pathState != 0) {
            updateAutoCycle();
        } else {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }

        follower.update();
    }

    private void updateAutoCycle() {
        switch(pathState) {
            case 1:
                if (follower.getCurrentTValue() > 0.9) {
                    CommandScheduler.getInstance().schedule(new AutoSpecimenGrab(robot.specimen));

                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 300) {

                    follower.followPath(wallToChamber);
                    setPathState(3);
                }
                break;

            case 3:
                if (follower.getCurrentTValue() > 0.97) {
                    CommandScheduler.getInstance().schedule(new AutoSpecimenDeposit(robot.specimen));
                    follower.followPath(chamberToWall);
                    setPathState(1);
                }
                break;
            case 4:
                if (follower.getCurrentTValue() > 0.8) {
                    follower.followPath(
                            follower.pathBuilder()
                                    .addPath(new BezierLine(follower.getPose(), humanPlayerDepositPose))
                                    .setTangentHeadingInterpolation()
                                    .setZeroPowerAccelerationMultiplier(10)
                                    .addParametricCallback(0.8, () -> robot.extension.setExtensionState(ExtensionSubsystem.ExtensionState.EXTENDED))
                                    .build()
                    );
                    setPathState(5);
                }
                break;
            case 5:
                if (follower.getCurrentTValue() > 0.6) {
                    schedule(
                        new SequentialCommandGroup(
                                new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.EJECT),
                                new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.EJECT),
                                new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.REVERSING),

                                new WaitCommand(100),

                                new IdleIntakeCommand(robot.intake),

                                //new ExtensionSetPosition(robot.extension, ExtensionSubsystem.ExtensionState.TRANSFER),
                                new ExtensionSetPosition_INST(robot.extension, ExtensionSubsystem.ExtensionState.CUSTOM)
                        )
                    );
                    setPathState(0);
                    follower.startTeleopDrive();
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}

