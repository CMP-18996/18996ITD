package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.commands.ColorSensorStateCommand;
import org.firstinspires.ftc.teamcode.common.commands.DepositRotationCommand;
import org.firstinspires.ftc.teamcode.common.commands.ExtendAndBeginIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.ExtendCommand;
import org.firstinspires.ftc.teamcode.common.commands.HangCommand;
import org.firstinspires.ftc.teamcode.common.commands.InstantLiftCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.RetractAndTransferClawsCommand;
import org.firstinspires.ftc.teamcode.common.commands.SingleColorSensorCommand;
import org.firstinspires.ftc.teamcode.common.commands.SpecimenArmCommand;
import org.firstinspires.ftc.teamcode.common.commands.SpecimenGripperCommand;
import org.firstinspires.ftc.teamcode.common.commands.TrapdoorCommand;
import org.firstinspires.ftc.teamcode.common.commands.ZeroMotorCommand;
import org.firstinspires.ftc.teamcode.common.robot.Drive;
import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;
import org.firstinspires.ftc.teamcode.common.robot.OdometryHardware;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.Team;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp
public class EarlyTeleOp extends CommandOpMode {
    private final Team team = Team.BLUE;
    private final boolean liftEnabled = false;
    private Team oppositeTeam;

    private Robot robot;
    private GamepadEx gamepad_1;
    private DcMotorEx hangMotor;

    private GamepadEx gamepad_2;
    private Drive drive;
    private OdometryHardware odometryHardware;
    private final double pickingUpVal  = IntakeSubsystem.IntakeRotatorState.PICKING_UP.val;

    public IntakeSubsystem.IntakingState previousIntakingState = IntakeSubsystem.IntakingState.DISABLED;

    @Override
    public void initialize() {
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hangMotor = hardwareMap.get(DcMotorEx.class, HardwareMapNames.HANG_MOTOR_1);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive = new Drive(hardwareMap);
        drive.setBreakMode(DcMotor.ZeroPowerBehavior.BRAKE);

        odometryHardware = new OdometryHardware(hardwareMap);

        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap, team, true, Subsystems.ALL);
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);

        // TODO CAHNGE HTIS TO GLOBAL
        if (team.equals(Team.RED)) oppositeTeam = Team.BLUE;
        else oppositeTeam = Team.RED;

        //CommandScheduler.getInstance().schedule(
         //       new ZeroMotorCommand(robot.extension, robot.lift)
        //);

        // MAIN DRIVER
        // MAIN DRIVER
        // MAIN DRIVER
        gamepad_1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new LiftSetPosition(robot.lift, LiftSubsystem.LiftState.HIGH_BASKET),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.READY_TO_DEPOSIT)
                        ),
                        new ScheduleCommand(
                                new LiftSetPosition(robot.lift, LiftSubsystem.LiftState.GROUND),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY)
                        ),
                        () -> robot.lift.getCurrTarget() != LiftSubsystem.LiftState.HIGH_BASKET.height
                )
        );

        gamepad_1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new LiftSetPosition(robot.lift, LiftSubsystem.LiftState.LOW_BASKET),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.READY_TO_DEPOSIT)
                        ),
                        new ScheduleCommand(
                                new LiftSetPosition(robot.lift, LiftSubsystem.LiftState.GROUND),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY)
                        ),
                        () -> robot.lift.getCurrTarget() != LiftSubsystem.LiftState.LOW_BASKET.height
                )
        );

        gamepad_2.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> {
                    robot.setTransferringState(true);
                    schedule(
                            new SequentialCommandGroup(
                                    /*
                                    new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED),
                                    new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.MOVING),
                                    new WaitCommand(300),
                                    new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                                    new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.CONTRACTED),
                                    new WaitCommand(500),
                                    new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING),
                                    new WaitCommand(1000),
                                    new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.MOVING),
                                    new WaitCommand(400),
                                    new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED),
                                    new ConditionalCommand(
                                            new InstantLiftCommand(robot.lift, LiftSubsystem.HIGH_BASKET),
                                            new WaitCommand(0),
                                            () -> liftEnabled
                                    ),

                                    new WaitCommand(100),
                                    new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.INTERMEDIATE),
                                    new WaitCommand(200),


                                    new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.CUSTOM)
                                     */
                                    new RetractAndTransferClawsCommand(robot.extension, robot.intake, robot.deposit, robot.lift),
                                    new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.CUSTOM),
                                    new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED)
                            ).whenFinished(() -> robot.setTransferringState(false))
                    );
                }
        );

        // A is Cross
        gamepad_1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.DEPOSITING),
                                new WaitCommand(500),
                                new WaitCommand(1000),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY),
                                new InstantLiftCommand(robot.lift, LiftSubsystem.LiftState.GROUND)
                        ),
                        new SequentialCommandGroup(
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.DEPOSITING),
                                new WaitCommand(500),
                                new WaitCommand(1000),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY)
                        ),
                        () -> robot.lift.getCurrTarget() != LiftSubsystem.LiftState.GROUND.height
                )
        );

        gamepad_1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.ACTIVE),
                                new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.CLOSED),
                                new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.PICKING_UP)
                        ),
                        new ScheduleCommand(
                                new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED),
                                new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.TRANSFERRING)
                        ),
                        () -> robot.intake.getIntakingState().equals(IntakeSubsystem.IntakingState.DISABLED)
                )
        );

        gamepad_1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                () -> {
                    previousIntakingState = robot.intake.getIntakingState();
                    schedule(
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING)
                    );
                }
        ).whenReleased(
                new IntakeCommand(robot.intake, previousIntakingState)
        );
        /*
        gamepad_1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new HangCommand(robot.hang, HangSubsystem.HangPosition.L3)
                        ),
                        new ScheduleCommand(
                                new HangCommand(robot.hang, HangSubsystem.HangPosition.DOWN)
                        ),
                        () -> robot.hang.getState().equals(HangSubsystem.HangPosition.DOWN)
                )
        ); */

        gamepad_1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.CLOSED)
                        ),
                        new ScheduleCommand(
                                new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.OPEN)
                        ),
                        () -> robot.specimen.getGripperPosition() != SpecimenSubsystem.GripperPosition.CLOSED
                )
        );

        gamepad_1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> {
            LiftSubsystem.LiftState.GROUND.changeHeight(10);
            robot.lift.setTargetPosition(LiftSubsystem.LiftState.GROUND);
        });
        gamepad_1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            LiftSubsystem.LiftState.GROUND.changeHeight(-10);
            robot.lift.setTargetPosition(LiftSubsystem.LiftState.GROUND);
        });
        gamepad_2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> {
            LiftSubsystem.LiftState.GROUND.changeHeight(10);
            robot.lift.setTargetPosition(LiftSubsystem.LiftState.GROUND);
        });
        gamepad_2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            LiftSubsystem.LiftState.GROUND.changeHeight(-10);
            robot.lift.setTargetPosition(LiftSubsystem.LiftState.GROUND);
        });

        //  SECOND DRIVER
        //  SECOND DRIVER
        //  SECOND DRIVER
        gamepad_2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.CLOSED)
                        ),
                        new ScheduleCommand(
                                new SpecimenGripperCommand(robot.specimen, SpecimenSubsystem.GripperPosition.OPEN)
                        ),
                        () -> robot.specimen.getGripperPosition() != SpecimenSubsystem.GripperPosition.CLOSED
                )
        );

        gamepad_2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                () -> robot.setTransferringState(false)
        );

        gamepad_2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ScheduleCommand(
                        new SpecimenArmCommand(robot.specimen, SpecimenSubsystem.SpecimenPosition.CHAMBER)
                )
        );

        gamepad_2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ScheduleCommand(
                        new SpecimenArmCommand(robot.specimen, SpecimenSubsystem.SpecimenPosition.WALL)
                )
        );

        gamepad_2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ScheduleCommand(
                        new ConditionalCommand(
                                new ColorSensorStateCommand(robot.intake, IntakeSubsystem.ColorSensorState.BROKEN),
                                new ColorSensorStateCommand(robot.intake, IntakeSubsystem.ColorSensorState.WORKING),
                                () -> robot.intake.getColorSensorState() == IntakeSubsystem.ColorSensorState.WORKING
                        )
                )
        );

        /*
        gamepad_2.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                ; // LIFT GOES UP WHEN HELD
        );

        gamepad_2.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                ; // LIFT GOES DOWN WHEN HELD
        );

         */
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        if (gamepad_2.getLeftY() != 0) {
            robot.specimen.manualAdjustArm((int) (Math.cbrt(gamepad_2.getLeftY())  * 10));
        }
        if (gamepad_2.getRightY() != 0) {
            robot.specimen.manualAdjustWrist(Math.cbrt(gamepad_2.getRightY() / 50));
        }


        if (robot.extension.getState().equals(ExtensionSubsystem.ExtensionState.CUSTOM)) {
            double rawExtensionPower;
            rawExtensionPower = gamepad_1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            robot.extension.setPower(Math.pow(rawExtensionPower, 3));
        }

        if (robot.intake.getIntakeRotatorState().equals(IntakeSubsystem.IntakeRotatorState.PICKING_UP)) {
            double adjustment = -gamepad1.right_stick_y * .2;
            robot.intake.slightlyIncrementRotator(adjustment);
        }


        if (gamepad2.circle) hangMotor.setPower(-1);
        else if (gamepad2.square) hangMotor.setPower(1);
        else hangMotor.setPower(0);

        //Team detectedColor = robot.intake.updateColorState2();
        Team detectedColor = robot.intake.updateColorState2();

        boolean acceptedYellow = (detectedColor.equals(Team.YELLOW) && liftEnabled);

        if ((detectedColor.equals(team) || acceptedYellow) && !robot.isTransferring() && robot.intake.getIntakeRotatorState() == IntakeSubsystem.IntakeRotatorState.PICKING_UP) {
            robot.setTransferringState(true);
            schedule(
                    new SequentialCommandGroup(
                            /*
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED),
                            new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.MOVING),
                            new WaitCommand(300),
                            new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                            new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.CONTRACTED),
                            new WaitCommand(500),
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING),
                            new SingleColorSensorCommand(robot.intake, IntakeSubsystem.ColorState.NONE),
                            new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.MOVING),
                            new WaitCommand(800),
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED),
                            new ConditionalCommand(
                                    new InstantLiftCommand(robot.lift, LiftSubsystem.HIGH_BASKET),
                                    new WaitCommand(0),
                                    () -> liftEnabled
                            ),

                            new WaitCommand(100),
                            new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.INTERMEDIATE),
                            new WaitCommand(200),


                            new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.CUSTOM)
                             */
                            new RetractAndTransferClawsCommand(robot.extension, robot.intake, robot.deposit, robot.lift),
                            new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.CUSTOM),
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED)
                    )//.whenFinished(() -> robot.setTransferringState(false))
            );
        }
        else if ((detectedColor.equals(oppositeTeam) || detectedColor.equals(Team.YELLOW)) && !robot.isTransferring()) {
        //else if ((detectedColor.equals(oppositeTeam) || detectedColor.equals(Team.YELLOW)) && !robot.isTransferring()) {
            schedule(
                    new SequentialCommandGroup(
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING),
                            new WaitCommand(400),
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.ACTIVE)
                    )
            );
        }

        if (gamepad1.options) {
            odometryHardware.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            //telemetry.addLine("Reset Angle!");
        }

        //odometryHardware.pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        //double heading = odometryHardware.pinpoint.getHeading();

        drive.robotCentricDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);



        telemetry.addData("Intake Rotator State:", robot.intake.getIntakeRotatorState());
        telemetry.addData("Intake Roller State:", robot.intake.getIntakingState());
        telemetry.addData("Trapdoor State:", robot.intake.getTrapdoorState());
        telemetry.addData("Detected Color:", detectedColor);
        telemetry.addData("Color RED:", robot.intake.colorSensor.red());
        telemetry.addData("Color GREEN:", robot.intake.colorSensor.green());
        telemetry.addData("Color BLUE:", robot.intake.colorSensor.blue());
        telemetry.addData("Color ALPHA:", robot.intake.colorSensor.alpha());
        telemetry.addData("Previous intaking state:", previousIntakingState);

        telemetry.addData("Transferring State:", robot.isTransferring());


        telemetry.addLine("");
        telemetry.addData("Extension State:", robot.extension.getState());
        telemetry.addData("Extension Target:", robot.extension.getTargetPosition());
        telemetry.addData("Extension Position:", robot.extension.getPosition());
        telemetry.addData("Extension Error:", robot.extension.error);
        telemetry.addData("Extension Power", robot.extension.telemetryPower);
        //telemetry.addData("Extension Encoder", robot.extension.get)



        telemetry.addLine("");
        telemetry.addData("Deposit Rotator State:", robot.deposit.getTransferRotatorState());

        telemetry.addLine("");
        //telemetry.addData("Lift State:", ); Lift has no states?
        telemetry.addData("Ground", LiftSubsystem.LiftState.GROUND.height);
        telemetry.addData("Lift Target:", robot.lift.getCurrTarget());
        telemetry.addData("Lift Position", robot.lift.getCurrentPosition());
        telemetry.addData("Lift Error:", robot.lift.getError());
        telemetry.addData("Lift Power", robot.lift.telemetryPower);
        telemetry.addData("P", LiftSubsystem.Kp);
        telemetry.addData("I", LiftSubsystem.Ki);
        telemetry.addData("D", LiftSubsystem.Kd);
        telemetry.addData("F", LiftSubsystem.Kf);
        telemetry.addData("INtegral", robot.lift.integralSum);



        telemetry.addLine("");
        telemetry.addData("Arm State:", robot.specimen.getSpecimenState());
        telemetry.addData("Arm Target:", robot.specimen.getSpecimenState().armPosition);
        telemetry.addData("Arm Position:", robot.specimen.armMotor.getCurrentPosition());
        telemetry.addData("Arm Error:", robot.specimen.getError());

        telemetry.addLine("");
        telemetry.addData("Wrist State:", robot.specimen.getSpecimenState());
        telemetry.addData("Wrist Position:", robot.specimen.getSpecimenState().wristPosition);

        telemetry.addLine("");
        telemetry.addData("Gripper State:", robot.specimen.getGripperPosition());


        //Method WRIET FOR COLOR SESNRO)

        telemetry.update();



        /*
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("power", robot.lift.powerTELE);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);

         */

    }
}
