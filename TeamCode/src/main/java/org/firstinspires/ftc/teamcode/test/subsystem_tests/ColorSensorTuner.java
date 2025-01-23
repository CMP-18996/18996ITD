package org.firstinspires.ftc.teamcode.test.subsystem_tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@TeleOp(name = "Color Sensor Tuner")
public class ColorSensorTuner extends LinearOpMode {

    ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, HardwareMapNames.INTAKE_COLOR_SENSOR);

    @Override
    public void runOpMode() {
        colorSensor.enableLed(true);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("RED", colorSensor.red());
            telemetry.addData("BLUE", colorSensor.blue());
            telemetry.addData("GREEN", colorSensor.green());
            telemetry.addData("ALPHA", colorSensor.alpha());
            telemetry.update();
        }
    }
}

