package org.firstinspires.ftc.teamcode.test.subsystem_tests;



import org.firstinspires.ftc.teamcode.common.commands.IntakeRotatorCommand;
import org.firstinspires.ftc.teamcode.common.commands.TrapdoorCommand;
import org.firstinspires.ftc.teamcode.common.robot.Robot;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.Subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Disabled
@Config
@TeleOp(name = "Color Sensor Tuner")
public class ColorSensorTuner extends CommandOpMode {

    Subsystems subsystems = Subsystems.INTAKE;
    Robot robot;
    ColorSensor colorSensor;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap, subsystems);
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        super.schedule(
                new SequentialCommandGroup(
//                        new IntakeCommand(robot.intake, IntakeSubsystem.IntakingState.ACTIVE),
                        new IntakeRotatorCommand(robot.intake, IntakeSubsystem.IntakeRotatorState.PICKING_UP),
                        new TrapdoorCommand(robot.intake, IntakeSubsystem.TrapdoorState.CLOSED)
                )
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("red:", colorSensor.red());
        telemetry.addData("blue:", colorSensor.blue());
        telemetry.addData("green:", colorSensor.green());
        telemetry.addData("alpha", colorSensor.alpha());
        telemetry.update();
    }
}

