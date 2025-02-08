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

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.commands.autoCommands.AutoSpecimenDeposit;
import org.firstinspires.ftc.teamcode.common.commands.autoCommands.AutoSpecimenGrab;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.CompactForHangCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.HumanPlayerDepositCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.ReadySampleDepositCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.RejectSampleCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.RetractAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.SampleDepositCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositTrapdoorPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.hang.HangCommand_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetColorSensorStatus_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetMotorState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeTrapdoorSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.lift.ZeroLift;
import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetArmPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetGripperPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.Drive;
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
    private final Team team = Team.RED;
    private boolean acceptYellow = true;
    private boolean liftEnabled = true;

    private Robot robot;
    private int pathState = 0;
    private Timer pathTimer;

    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;

    public IntakeSubsystem.IntakeMotorState previousIntakingState = IntakeSubsystem.IntakeMotorState.DISABLED;

    private final Pose chamberPose = new Pose(38, 66, Math.toRadians(0));

    private final Pose wallPose = new Pose(7.5, 28, 0);

    private double previousPower = 0;
    private double previousLeftY = 0;

    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    private boolean isCycling = false;

    private PathChain wallToChamber, chamberToWall;

    public void runOpMode() throws InterruptedException {
        waitForStart();
        reset();
        initialize();

        pathTimer.resetTimer();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
    }

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, team, acceptYellow, Subsystems.ALL);

        pathTimer = new Timer();

//        drive = new Drive(hardwareMap);
  //      drive.setBreakMode(DcMotor.ZeroPowerBehavior.BRAKE);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

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

        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);

        // MAIN DRIVER
        // MAIN DRIVER
        // MAIN DRIVER

        // Lift High Bucket
        gamepad_1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.HIGH_BUCKET)
                        ),
                        new ScheduleCommand(
                                new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.TRANSFER),
                                new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.TRANSFER)
                        ),
                        () -> !robot.lift.getLiftState().equals(LiftSubsystem.LiftState.HIGH_BUCKET)
                )
        );

        // Lift Low Bucket
        gamepad_1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.LOW_BUCKET)
                        ),
                        new ScheduleCommand(
                                new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.TRANSFER),
                                new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.TRANSFER)
                        ),
                        () -> !robot.lift.getLiftState().equals(LiftSubsystem.LiftState.LOW_BUCKET)
                )
        );

        gamepad_1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                () -> {
                    schedule(
                            new TransferSampleCommand(robot.intake)
                    );
                }
        );

        gamepad_1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                () -> {
                    schedule(
                            new RejectSampleCommand(robot.intake),
                            new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.PICK_UP),
                            new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.PICK_UP)
                    );
                }
        );

        // AUTO SPECIMEN CYCLE
        gamepad_1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> {
                    if(!isCycling) {
                        follower.setPose(new Pose(7.5, 28, 0));
                        follower.followPath(chamberToWall);
                        setPathState(1);

                        /*
                        schedule(
                                new InstantCommand(() -> )),
                                new InstantCommand(() -> setPathState(1))
                        );

                         */
                    } else {
                        follower.startTeleopDrive();
                        setPathState(0);
                    }
                }
        );

        // Enable/Disable Intake
        gamepad_1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.PICK_UP),
                                new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.PICK_UP),
                                new IntakeSetMotorState_INST(robot.intake, IntakeSubsystem.IntakeMotorState.ACTIVE)
                        ),
                        new ScheduleCommand(
                                new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.MOVING),
                                new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.MOVING),
                                new IntakeSetMotorState_INST(robot.intake, IntakeSubsystem.IntakeMotorState.HOLD)
                        ),
                        () -> !robot.intake.getIntakeArmState().equals(IntakeSubsystem.IntakeArmState.PICK_UP)
                )
        );

        // Deposit button kinda idk
        // TODO ADD MANUAL REVERSE
        // nesting conditional commands what the fuck ftclib
        gamepad_1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new ConditionalCommand(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new ReadySampleDepositCommand(robot.deposit),
                                        new WaitCommand(1000),
                                        new SampleDepositCommand(robot.deposit)
                                ),
                                new SequentialCommandGroup(
                                        new SampleDepositCommand(robot.deposit),
                                        new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.TRANSFER)
                                ),
                                () -> robot.lift.getLiftState().equals(LiftSubsystem.LiftState.TRANSFER)
                        ),
                        new SequentialCommandGroup(
                                new HumanPlayerDepositCommand(robot.deposit, robot.lift),
                                new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.TRANSFER)
                        ),
                        () -> !robot.lift.getLiftState().equals(LiftSubsystem.LiftState.HUMAN_PLAYER_DEPOSIT)
                )
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

        // Set Intake to Rest
        gamepad_1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                () -> {
                    schedule(
                            new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.REST),
                            new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.REST),
                            new IntakeSetMotorState_INST(robot.intake, IntakeSubsystem.IntakeMotorState.DISABLED)
                    );
                }
        );

        // Set Intake to Moving
        gamepad_1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                () -> {
                    schedule(
                            new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.MOVING),
                            new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.MOVING),
                            new IntakeSetMotorState_INST(robot.intake, IntakeSubsystem.IntakeMotorState.HOLD)
                    );
                }
        );

        // TODO FIX
        // EMERGENCY RESET
        gamepad_1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                () -> {
                    gamepad1.rumbleBlips(3);
                    //CommandScheduler.getInstance().reset();
                    //robot.setTransferringState(false);
                    schedule(
                            new SequentialCommandGroup(
                                new ExtensionSetPosition_INST(robot.extension, ExtensionSubsystem.ExtensionState.CUSTOM),
                                new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.TRANSFER),
                                new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.REST),
                                new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.REST),
                                new IntakeSetMotorState_INST(robot.intake, IntakeSubsystem.IntakeMotorState.DISABLED),
                                new IntakeTrapdoorSetPosition_INST(robot.intake, IntakeSubsystem.IntakeTrapdoorState.CLOSED)
                            )
                    );
                }
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
                () -> {
                    schedule(
                            new HangCommand_INST(robot.hang, HangSubsystem.HangState.UP)
                    );
                }
        ).whenReleased(
                () -> {
                    schedule(
                            new HangCommand_INST(robot.hang, HangSubsystem.HangState.OFF)
                    );
                });

        gamepad_2.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                () -> {
                    schedule(
                            new HangCommand_INST(robot.hang, HangSubsystem.HangState.DOWN)
                    );
                }
        ).whenReleased(
                () -> {
                    schedule(
                            new HangCommand_INST(robot.hang, HangSubsystem.HangState.HOLD)
                    );
                });

        gamepad_2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                () -> {
                    acceptYellow = !acceptYellow;
                    robot.setAcceptYellow(acceptYellow);
                }
        );

        gamepad_2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                () -> {
                    liftEnabled = !liftEnabled;
                }
        );

        gamepad_2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> {
                    robot.setTransferringState(true);
                    if (liftEnabled) {
                        schedule(
                                new SequentialCommandGroup(
                                        new RetractAndTransferCommand(robot.extension, robot.intake, robot.deposit, robot.lift),
                                        new ExtensionSetPosition_INST(robot.extension, ExtensionSubsystem.ExtensionState.CUSTOM),
                                        new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.HIGH_BUCKET),
                                        new InstantCommand(() -> robot.setTransferringState(false))
                                )
                        );
                    }
                    else {
                        schedule(
                                new SequentialCommandGroup(
                                        new RetractAndTransferCommand(robot.extension, robot.intake, robot.deposit, robot.lift),
                                        new ExtensionSetPosition_INST(robot.extension, ExtensionSubsystem.ExtensionState.CUSTOM),
                                        new InstantCommand(() -> robot.setTransferringState(false))
                                )
                        );
                    }

                }
        );

        gamepad_2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                () -> {
                    schedule(
                            new CompactForHangCommand(robot.extension, robot.intake, robot.deposit, robot.lift, robot.specimen)
                    );
                }
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        if (gamepad_1.getRightY() != previousLeftY) {
            robot.intake.adjustWristPosition(gamepad_1.getRightY() / 10);
            previousLeftY = gamepad_1.getRightY();
        }

        // Extension Triggers
        double power = Math.pow(gamepad_1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), 3);

        if(power != previousPower) {
            robot.extension.setExtensionMotorPower(power);
            previousPower = power;
        }

        // Auto Transfer
        IntakeSubsystem.Color detectedColor = robot.intake.getCurrentColor();

        if(!detectedColor.equals(IntakeSubsystem.Color.NONE) && !robot.isTransferring()) {
            if(robot.acceptColor(detectedColor)) {
                gamepad1.rumbleBlips(1);
                robot.setTransferringState(true);
                if (liftEnabled) {
                    schedule(
                            new SequentialCommandGroup(
                                    new RetractAndTransferCommand(robot.extension, robot.intake, robot.deposit, robot.lift),
                                    new ExtensionSetPosition_INST(robot.extension, ExtensionSubsystem.ExtensionState.CUSTOM),
                                    new ZeroLift(robot.lift),
                                    new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.HIGH_BUCKET),
                                    new ReadySampleDepositCommand(robot.deposit),
                                    new InstantCommand(() -> robot.setTransferringState(false))
                            )
                    );
                }
                else {
                    schedule(
                            new SequentialCommandGroup(
                                    new RetractAndTransferCommand(robot.extension, robot.intake, robot.deposit, robot.lift),
                                    new ExtensionSetPosition_INST(robot.extension, ExtensionSubsystem.ExtensionState.CUSTOM),
                                    new ZeroLift(robot.lift),
                                    new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.HUMAN_PLAYER_DEPOSIT),
                                    new InstantCommand(() -> robot.setTransferringState(false)),
                                    new DepositTrapdoorPosition_INST(robot.deposit, DepositSubsystem.DepositTrapdoorState.CLOSED)
                            )
                    );
                }
            }
            else {
                gamepad1.rumbleBlips(2);
                schedule(
                        new SequentialCommandGroup(
                                new RejectSampleCommand(robot.intake)
                        )
                );
            }
        }

        telemetry.addData("COLOR", detectedColor);
        telemetry.addData("COLOR SENSOR STATUS", robot.intake.getColorSensorStatus());
        telemetry.addData("ACCEPT YELLOW", acceptYellow);
        telemetry.addData("LIFT ENABLED", liftEnabled);
        telemetry.addData("TRANSFERRING STATE", robot.isTransferring());
        telemetry.addData("Accept Color", robot.acceptColor(detectedColor));
        telemetry.addData("LIFT STATE", robot.lift.getLiftState());
        telemetry.update();

        if((-gamepad1.left_stick_y == 0 || -gamepad1.left_stick_x == 0 || gamepad1.right_stick_x == 0) && pathState != 0) {
            updateAutoCycle();
        } else {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }
        follower.update();
    }

    private void updateAutoCycle() {
        switch(pathState) {
            case 1:
                if (follower.getCurrentTValue() > 0.92) {
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
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

}

