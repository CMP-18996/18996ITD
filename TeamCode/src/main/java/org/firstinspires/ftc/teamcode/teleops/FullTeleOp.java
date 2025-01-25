package org.firstinspires.ftc.teamcode.teleops;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.commands.complexCommands.RejectSampleCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.RetractAndTransferCommand;
import org.firstinspires.ftc.teamcode.common.commands.complexCommands.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.common.commands.deposit.DepositSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.extension.ExtensionSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeArmSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetColorSensorStatus_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeSetRollerState_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeTrapdoorSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.intake.IntakeWristSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.lift.LiftSetPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetArmPosition_INST;
import org.firstinspires.ftc.teamcode.common.commands.specimen.SpecimenSetGripperPosition_INST;
import org.firstinspires.ftc.teamcode.common.robot.Drive;
import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.Team;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp
public class FullTeleOp extends CommandOpMode {
    private final Team team = Team.RED;
    private boolean acceptYellow = true;
    private boolean liftEnabled = true;

    private Robot robot;
    private Drive drive;

    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;

    private DcMotorEx hangMotor;

    public IntakeSubsystem.IntakeRollerState previousIntakingState = IntakeSubsystem.IntakeRollerState.DISABLED;

    private double power = 0;
    private double previousPower = 0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap, team, acceptYellow, Subsystems.ALL);

        hangMotor = hardwareMap.get(DcMotorEx.class, HardwareMapNames.HANG_MOTOR_1);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive = new Drive(hardwareMap);
        drive.setBreakMode(DcMotor.ZeroPowerBehavior.BRAKE);

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

        // Deposit
        gamepad_1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> {
                    schedule(
                            new SequentialCommandGroup(
                                    new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.DEPOSIT),
                                    new WaitCommand(600),
                                    new DepositSetPosition_INST(robot.deposit, DepositSubsystem.BucketState.TRANSFER),
                                    new WaitCommand(600),
                                    new LiftSetPosition_INST(robot.lift, LiftSubsystem.LiftState.TRANSFER)
                            )
                    );
                }
        );

        // Enable/Disable Intake
        gamepad_1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.PICK_UP),
                                new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.PICK_UP),
                                new IntakeSetRollerState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.ACTIVE)
                        ),
                        new ScheduleCommand(
                                new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.MOVING),
                                new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.MOVING),
                                new IntakeSetRollerState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.ACTIVE)
                        ),
                        () -> !robot.intake.getIntakeArmState().equals(IntakeSubsystem.IntakeArmState.PICK_UP)
                )
        );

        // Reversing Intake
        gamepad_1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                () -> {
                    previousIntakingState = robot.intake.getIntakeRollerState();
                    schedule(
                            new IntakeSetRollerState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.REVERSING)
                    );
                }
        ).whenReleased(
                new IntakeSetRollerState_INST(robot.intake, previousIntakingState)
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
                            new IntakeSetRollerState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.DISABLED)
                    );
                }
        );

        // Set Intake to Moving
        gamepad_1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                () -> {
                    schedule(
                            new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.MOVING),
                            new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.MOVING),
                            new IntakeSetRollerState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.ACTIVE)
                    );
                }
        );

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
                                new IntakeSetRollerState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.DISABLED),
                                new IntakeTrapdoorSetPosition_INST(robot.intake, IntakeSubsystem.TrapdoorState.CLOSED)
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
                () -> hangMotor.setPower(1.0)
        ).whenReleased(() -> hangMotor.setPower(0.0));

        gamepad_2.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                () -> hangMotor.setPower(-1.0)
        ).whenReleased(() -> hangMotor.setPower(0.0));

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
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        // Manual Specimen Adjustments
        if (gamepad_2.getLeftY() != 0) {
            robot.specimen.manualAdjustArm((int) (Math.cbrt(gamepad_2.getLeftY() * 2)));
        }
        if (gamepad_2.getRightY() != 0) {
            robot.specimen.manualAdjustWrist(Math.cbrt(gamepad_2.getRightY() / 100));
        }

        // Extension Triggers
        power = Math.pow(gamepad_1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), 3);

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
            else {
                gamepad1.rumbleBlips(2);
                previousIntakingState = robot.intake.getIntakeRollerState();
                schedule(
                        new SequentialCommandGroup(
                                /*
                            new RejectSampleCommand(robot.intake),
                            new IntakeArmSetPosition_INST(robot.intake, IntakeSubsystem.IntakeArmState.PICK_UP),
                            new IntakeWristSetPosition_INST(robot.intake, IntakeSubsystem.IntakeWristState.PICK_UP),
                            new IntakeSetRollerState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.ACTIVE)

                                 */
                            new IntakeSetRollerState_INST(robot.intake, IntakeSubsystem.IntakeRollerState.REVERSING),
                            new WaitCommand(300),
                            new IntakeSetRollerState_INST(robot.intake, previousIntakingState)
                            //new IntakeSetRollerState_INST(robot.intake, previousIntakingState) // This works???? wtf
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
        telemetry.update();

        drive.robotCentricDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
}

