package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.commands.DepositRotationCommand;
import org.firstinspires.ftc.teamcode.common.commands.ExtendCommand;
import org.firstinspires.ftc.teamcode.common.commands.HangCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.commands.LiftSetPosition;
import org.firstinspires.ftc.teamcode.common.commands.TrapdoorCommand;
import org.firstinspires.ftc.teamcode.common.robot.Drive;
import org.firstinspires.ftc.teamcode.common.robot.OdometryHardware;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.Team;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;

@TeleOp
public class EarlyTeleOp extends CommandOpMode {
    private Team team = Team.RED;
    private Team oppositeTeam;

    private Robot robot;
    private GamepadEx gamepad;
    private GamepadEx subsystem_gamepad;
    private boolean alreadyTrans = false;
    private Drive drive;
    private OdometryHardware odometryHardware;
    private final double pickingUpVal  = IntakeSubsystem.IntakeRotatorState.PICKING_UP.val;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new Drive(hardwareMap);
        drive.setBreakMode(DcMotor.ZeroPowerBehavior.BRAKE);

        odometryHardware = new OdometryHardware(hardwareMap);

        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap, Subsystems.ALL);
        gamepad = new GamepadEx(gamepad1);
        subsystem_gamepad = new GamepadEx(gamepad2);

        if (team.equals(Team.RED)) oppositeTeam = Team.BLUE;
        else oppositeTeam = Team.RED;

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new LiftSetPosition(robot.lift, LiftSubsystem.HIGH_BASKET),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.READY_TO_DEPOSIT)
                        ),
                        new ScheduleCommand(
                                new LiftSetPosition(robot.lift, LiftSubsystem.GROUND),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY)
                        ),
                        () -> robot.lift.getCurrTarget() != LiftSubsystem.HIGH_BASKET
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new LiftSetPosition(robot.lift, LiftSubsystem.LOW_BASKET),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.READY_TO_DEPOSIT)
                        ),
                        new ScheduleCommand(
                                new LiftSetPosition(robot.lift, LiftSubsystem.GROUND),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY)
                        ),
                        () -> robot.lift.getCurrTarget() != LiftSubsystem.LOW_BASKET
                )
        );

        // A is Cross
        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.DEPOSITING),
                                new WaitCommand(800),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY),
                                new LiftSetPosition(robot.lift, LiftSubsystem.GROUND)
                        ),
                        new SequentialCommandGroup(
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.DEPOSITING),
                                new WaitCommand(800),
                                new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.TRANSFER_READY)
                        ),
                        () -> robot.lift.getCurrTarget() != LiftSubsystem.GROUND
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
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

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING)
        ).whenReleased(
                new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ConditionalCommand(
                        new ScheduleCommand(
                                new HangCommand(robot.hang, HangSubsystem.HangPosition.L3)
                        ),
                        new ScheduleCommand(
                                new HangCommand(robot.hang, HangSubsystem.HangPosition.DOWN)
                        ),
                        () -> robot.hang.getState().equals(HangSubsystem.HangPosition.DOWN)
                )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        robot.hang.hangMotor.setPower(subsystem_gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - subsystem_gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        if (robot.extension.getState().equals(ExtensionSubsystem.ExtensionState.CUSTOM)){
            double rawExtensionPower = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            robot.extension.setPower(Math.pow(rawExtensionPower, 3));
        }

        if (robot.intake.getIntakeRotatorState().equals(IntakeSubsystem.IntakeRotatorState.PICKING_UP)) {
            double adjustment = -gamepad.getRightY() * .08;
            robot.intake.slightlyIncrementRotator(adjustment);
        }


        //Team detectedColor = robot.intake.updateColorState2();
        Team detectedColor = robot.intake.updateColorState2();

        if ((detectedColor.equals(team) || detectedColor.equals(Team.YELLOW)) && !alreadyTrans) {
            alreadyTrans = true;

            schedule(
                    new SequentialCommandGroup(
                            new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.MOVING),
                            new WaitCommand(300),
                            new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.CONTRACTED),
                            new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.TRANSFERRING),
                            new WaitCommand(500),
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.REVERSING),
                            new WaitCommand(1000),
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED),
                            new LiftSetPosition(robot.lift, LiftSubsystem.HIGH_BASKET),
                            new DepositRotationCommand(robot.deposit, DepositSubsystem.TransferRotatorState.READY_TO_DEPOSIT),
                            new ExtendCommand(robot.extension, ExtensionSubsystem.ExtensionState.CUSTOM)
                    ).whenFinished(() -> alreadyTrans = false)
            );
        }

        else if (detectedColor.equals(oppositeTeam) && !alreadyTrans) {
            schedule(
                    new SequentialCommandGroup(
                            new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.DISABLED),
                            new WaitCommand(200),
                            new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.CLOSED)
                    )
            );
        }

        if (gamepad1.options) {
            odometryHardware.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            telemetry.addLine("Reset Angle!");
        }

        //odometryHardware.pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        //double heading = odometryHardware.pinpoint.getHeading();

        drive.robotCentricDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        /*
        telemetry.addData("Intake Rotator State:", robot.intake.getIntakeRotatorState());
        telemetry.addData("Intake Roller State:", robot.intake.getIntakingState());
        telemetry.addData("Trapdoor State:", robot.intake.getTrapdoorState());
        telemetry.addData("Detected Color:", detectedColor);
        telemetry.addData("Color RED:", robot.intake.colorSensor.red());
        telemetry.addData("Color GREEN:", robot.intake.colorSensor.green());
        telemetry.addData("Color BLUE:", robot.intake.colorSensor.blue());
        telemetry.addData("Color ALPHA:", robot.intake.colorSensor.alpha());

        telemetry.addData("alreadyTrans Value:", alreadyTrans);

        telemetry.addLine("");
        telemetry.addData("Extension State:", robot.extension.getState());
        telemetry.addData("Extension Target:", robot.extension.getTargetPosition());
        telemetry.addData("Extension Position:", robot.extension.getPosition());
        telemetry.addData("Extension Error:", robot.extension.getAbsError());
        telemetry.addData("Extension Power", robot.extension.telemetryPower);
        //telemetry.addData("Extension Encoder", robot.extension.get)

         */

        telemetry.addLine("");
        telemetry.addData("Deposit Rotator State:", robot.deposit.getTransferRotatorState());

        telemetry.addLine("");
        //telemetry.addData("Lift State:", ); Lift has no states?
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

        telemetry.addLine("");
        telemetry.addData("Hang State:", robot.hang.getState());
        telemetry.addData("Hang Target:", robot.hang.getTargetPosition());
        telemetry.addData("Hang Position", robot.hang.getCurrentPosition());

        telemetry.update();

        /*

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("power", robot.lift.powerTELE);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);

         */

    }
}
