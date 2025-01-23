package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

@TeleOp(name = "Color Sensor Tuner")
public class ColorSensorTuner extends LinearOpMode {

    private ColorSensor colorSensor;
    private IntakeSubsystem.Color currentColor;
    private IntakeSubsystem.ColorSensorStatus colorSensorStatus = IntakeSubsystem.ColorSensorStatus.DISABLED;
    private static int ALPHA_CUTOFF = 170;
    private double r, g, b, a;
    private int color;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, HardwareMapNames.INTAKE_COLOR_SENSOR);
        colorSensor.enableLed(true);

        waitForStart();

        while(opModeIsActive()) {
            updateCurrentColor();
            telemetry.addData("RED", r);
            telemetry.addData("BLUE", b);
            telemetry.addData("GREEN", g);
            telemetry.addData("ALPHA", a);
            telemetry.addData("COLOR", currentColor);
            telemetry.addData("color", color);
            telemetry.update();
        }
    }

    private void updateCurrentColor() {
        color = colorSensor.argb();
        a = (color >> 24) & 0xFF;
        r = (color >> 16) & 0xFF;
        g = (color >> 8) & 0xFF;
        b = color & 0xFF;

        if(colorSensorStatus.equals(IntakeSubsystem.ColorSensorStatus.DISABLED) || a < ALPHA_CUTOFF) {
            currentColor = IntakeSubsystem.Color.NONE;
        }
        else if(r > g && r > b){
            currentColor = IntakeSubsystem.Color.RED;
        }
        else if(g > r && g > b){
            currentColor = IntakeSubsystem.Color.YELLOW;
        }
        else if(b > r && b > g){
            currentColor = IntakeSubsystem.Color.BLUE;
        }
        else {
            // Pray this never happens
            currentColor = IntakeSubsystem.Color.NONE;
        }
    }
}

