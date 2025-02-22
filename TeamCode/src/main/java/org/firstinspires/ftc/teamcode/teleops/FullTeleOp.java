package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.google.gson.internal.bind.SqlDateTypeAdapter;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.autoCommands.AutoSpecimenDeposit;
import org.firstinspires.ftc.teamcode.common.commands.autoCommands.AutoSpecimenGrab;
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
import org.firstinspires.ftc.teamcode.common.robot.Team;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp
public class FullTeleOp extends CommandOpMode {
    private Robot robot;
    private int pathState = 0;
    private Timer pathTimer;
    private Color specimenColor = Color.BLUE;
    private boolean transferYellow = false;
    //private boolean transferring = false;

    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;

    private final Pose wallPose = new Pose(7.5, 28, Math.toRadians(0));

    private final Pose chamberPose = new Pose(38, 68, Math.toRadians(0));

    private final Pose specControl1 = new Pose(23.21, 73.87);
    private final Pose specControl2 = new Pose(24.46, 42.18);

    private final Pose humanPlayerDepositPose = new Pose(36, 34, Math.toRadians(-169));
    private final Pose humanPlayerDepositIntermediate = new Pose(57, 38, Math.toRadians(-169));

    private double previousPower = 0;

    private Follower follower;

    private PathChain wallToChamber, chamberToWall, humanPlayerDeposit;

    public void runOpMode() throws InterruptedException {
        waitForStart();
        reset();
        initialize();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
    }

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, Subsystems.ALL);

        robot.specimen.setSpecimenArmState(SpecimenSubsystem.SpecimenArmState.WALL);

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

        humanPlayerDeposit = follower.pathBuilder()
                .addPath(new BezierLine(new Point(humanPlayerDepositIntermediate), new Point(humanPlayerDepositPose)))
                .setTangentHeadingInterpolation()
                .setZeroPowerAccelerationMultiplier(10)
                .build();

        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);

        // MAIN DRIVER
        // MAIN DRIVER
        // MAIN DRIVER

        // Lift High Bucket
        gamepad_1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.HIGH_BUCKET),
                                new DepositTrapdoorPosition_INST(robot.deposit, DepositSubsystem.DepositTrapdoorState.CLOSED)
                        ),
                        new ScheduleCommand(
                                new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.TRANSFER),
                                new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.TRANSFER)
                        ),
                        () -> !robot.lift.getLiftState().equals(LiftSubsystem.LiftState.HIGH_BUCKET)
                )
        );

        // MANUAL HP DEPOSIT
        gamepad_1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                        new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.EJECT),
                        new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.EJECT),

                        new WaitCommand(200),

                        new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.REVERSING),

                        new WaitCommand(1000),

                        new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.DISABLED),
                        new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.MOVING),
                        new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.MOVING)
                )
        );

        gamepad_1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                () -> {
                    // manual transfer? or idk override
                }
        );

        // deposit deposit
        gamepad_1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new SequentialCommandGroup(
                        new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.DEPOSIT),
                        new WaitCommand(500),
                        // open depposit gate
                        new WaitCommand(500),
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
                        new ScheduleCommand(
                                new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.CLOSED)
                        ),
                        new ScheduleCommand(
                                new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.OPEN)
                        ),
                        () -> robot.specimen.getSpecimenGripperState().equals(SpecimenSubsystem.SpecimenGripperState.OPEN)
                )
        );

        // Intake el block 🥸🥸 !!!!!!
        gamepad_1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                        new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.ACTIVE),
                        new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.PICK_UP),
                        new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.PICK_UP)
                )
        ).whenReleased(
                () -> {
                    if(robot.intake.getCurrentColor().equals(Color.NONE)) {
                        schedule(
                            new SequentialCommandGroup(
                                    new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.DISABLED),
                                    new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.MOVING),
                                    new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.MOVING)
                            )
                        );
                    }
                }
        );

        // fuck ftclib
        // INTAKE PIVOT, deposit sample, deposit specimen
        gamepad_1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                () -> {
                    if(robot.intake.getIntakeWristState().equals(IntakeSubsystem.IntakeWristState.EJECT)) {
                        schedule(
                                new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.REVERSING)
                        );
                    }
                    else if(!robot.lift.getLiftState().equals(LiftSubsystem.LiftState.TRANSFER)) {
                        schedule(
                                // open deposit gate
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
                        schedule(
                                new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.DISABLED),
                                new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.MOVING),
                                new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.MOVING)
                        );
                    }
                    else if(!robot.lift.getLiftState().equals(LiftSubsystem.LiftState.TRANSFER)) {
                        schedule(
                                new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.TRANSFER)
                                //reset deposit
                        );
                    }
                    else {
                            // 45 logic, if needed
                    }
                }
        );

        // Auto hp deposit
        gamepad_1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                () -> {
                    follower.holdPoint(humanPlayerDepositIntermediate);
                    robot.extension.setExtensionState(ExtensionSubsystem.ExtensionState.TRANSFER);
                    setPathState(4);
                }
        );

        // FLOOR GRAB
        gamepad_1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new SequentialCommandGroup(
                        new IntakePivotSetPosition_INST(robot.intake, IntakeSubsystem.IntakePivotState.PIVOT_0),
                        new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.ACTIVE),
                        new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.FLOOR),
                        new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.FLOOR),

                        new WaitCommand(500),

                        new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.HOLD),
                        new IntakePivotSetPosition_INST(robot.intake, IntakeSubsystem.IntakePivotState.PIVOT_0),
                        new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.EJECT),
                        new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.EJECT)
                )
        );

        //  SECOND DRIVER
        //  SECOND DRIVER
        //  SECOND DRIVER
        gamepad_2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.CLOSED)
                        ),
                        new ScheduleCommand(
                                new SpecimenSetGripperPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenGripperState.OPEN)
                        ),
                        () -> robot.specimen.getSpecimenGripperState().equals(SpecimenSubsystem.SpecimenGripperState.OPEN)
                )
        );

        // Specimen Arm to Chamber
        gamepad_2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ScheduleCommand(
                        new SpecimenSetArmPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenArmState.CHAMBER)
                )
        );

        // Specimen Arm to Wall
        gamepad_2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ScheduleCommand(
                        new SpecimenSetArmPosition_INST(robot.specimen, SpecimenSubsystem.SpecimenArmState.WALL)
                )
        );

        // Enable/Disable Color Sensor
        gamepad_2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ScheduleCommand(
                        new ConditionalCommand(
                                new ScheduleCommand(
                                        new IntakeSetColorSensorStatus_INST(robot.intake, IntakeSubsystem.ColorSensorStatus.DISABLED)
                                ),
                                new ScheduleCommand(
                                        new IntakeSetColorSensorStatus_INST(robot.intake, IntakeSubsystem.ColorSensorStatus.ENABLED)
                                ),
                                () -> robot.intake.getColorSensorStatus().equals(IntakeSubsystem.ColorSensorStatus.ENABLED)
                        )
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
        CommandScheduler.getInstance().run();

        // Extension Triggers
        double power = 0.85 * Math.pow(gamepad_1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), 3) + 0.1;

        if(power != previousPower) {
            robot.extension.setExtensionMotorPower(power);
            previousPower = power;
        }

        if(!robot.intake.getCurrentColor().equals(Color.NONE) &&
                !robot.intake.getCurrentColor().equals(robot.intake.getPreviousColor())) {

            if (robot.intake.getCurrentColor().equals(Color.YELLOW)) {
                if (transferYellow) {
                    //trasnfer
                } else {
                    schedule(
                            new SequentialCommandGroup(
                                    new IntakePivotSetPosition_INST(robot.intake, IntakeSubsystem.IntakePivotState.PIVOT_0),
                                    new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.EJECT),
                                    new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.EJECT),

                                    new WaitCommand(500),

                                    new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.HOLD)
                            )
                    );
                }
            } else if (robot.intake.getCurrentColor().equals(specimenColor)) {
                schedule(
                        new SequentialCommandGroup(
                                new IntakePivotSetPosition_INST(robot.intake, IntakeSubsystem.IntakePivotState.PIVOT_0),
                                new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.EJECT),
                                new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.EJECT),

                                new WaitCommand(500),

                                new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.HOLD)
                        )
                );
            } else {
                schedule(
                        new SequentialCommandGroup(
                                new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.EJECT),
                                new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.REVERSING),

                                new WaitCommand(100),

                                new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.DISABLED),
                                new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.MOVING),
                                new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.MOVING)
                        )
                );
            }
        }

        telemetry.addData("PATH STATE", pathState);
        telemetry.addData("X VEL", follower.poseUpdater.getVelocity().getXComponent());
        telemetry.addData("Y VEL", follower.poseUpdater.getVelocity().getYComponent());
        telemetry.addData("H VEL", follower.poseUpdater.getAngularVelocity());
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
                if (follower.getPose().roughlyEquals(humanPlayerDepositIntermediate, 1)) { // idk if this will work
                    follower.followPath(humanPlayerDeposit);
                    robot.extension.setExtensionState(ExtensionSubsystem.ExtensionState.EXTENDED);
                    setPathState(5);
                }
                break;
            case 5:
                if (follower.getCurrentTValue() > 0.9) {
                    schedule(
                        new SequentialCommandGroup(
                                new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.EJECT),
                                new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.EJECT),
                                new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.REVERSING),

                                new WaitCommand(500),

                                new IntakeRollerSetState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.DISABLED),
                                new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.MOVING),
                                new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.MOVING),

                                new ExtensionSetPosition(robot.extension, ExtensionSubsystem.ExtensionState.TRANSFER),
                                new ExtensionSetPosition_INST(robot.extension, ExtensionSubsystem.ExtensionState.CUSTOM)
                        )
                    );
                    setPathState(1);
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

