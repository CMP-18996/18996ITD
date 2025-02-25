package org.firstinspires.ftc.teamcode.test.odo_tests;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Drawing;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@TeleOp(name = "Specimen Ultrasonic Test")
public class SpecimenUltrasonicTest extends LinearOpMode {

    private Limelight3A limelight;
    private AnalogInput rightSideUltrasonic;
    private AnalogInput backSideUltrasonic;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        /*
        limelight = hardwareMap.get(Limelight3A.class, HardwareMapNames.LIMELIGHT);
        limelight.pipelineSwitch(6);
        limelight.start();

         */

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, HardwareMapNames.PINPOINT);
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();

        rightSideUltrasonic = hardwareMap.get(AnalogInput.class, HardwareMapNames.RIGHT_SIDE_ULTRASONIC);
        backSideUltrasonic =  hardwareMap.get(AnalogInput.class, HardwareMapNames.BACK_SIDE_ULTRASONIC);

        waitForStart();

        while(opModeIsActive()) {
            pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);

            /*
            telemetry.addData("RIGHT SIDE VOLTAGE", rightSideUltrasonic.getVoltage());
            telemetry.addData("BACK SIDE VOLTAGE", backSideUltrasonic.getVoltage());
            */

            telemetry.addData("RIGHT SIDE", getDistanceFromVoltage(rightSideUltrasonic.getVoltage()));
            telemetry.addData("BACK SIDE", getDistanceFromVoltage(backSideUltrasonic.getVoltage()));

            Pose pose = new Pose(
                    getDistanceFromVoltage(backSideUltrasonic.getVoltage()) * Math.sin(Math.PI/2 - Math.abs(pinpoint.getHeading())),
                    (getDistanceFromVoltage(rightSideUltrasonic.getVoltage()) + 8) * Math.sin(Math.PI/2 - Math.abs(pinpoint.getHeading())),
                    pinpoint.getHeading());

            Drawing.drawRobot(pose, "#4CAF50");
            Drawing.sendPacket();

            telemetry.addData("x", pose.getX());
            telemetry.addData("y", pose.getY());
            telemetry.addData("heading", Math.toDegrees(pose.getHeading()));

            telemetry.update();
        }
    }

    private static double getDistanceFromVoltage(double voltage) {
        return voltage / 3.3 * 520 / 2.54 ;
    }
}
