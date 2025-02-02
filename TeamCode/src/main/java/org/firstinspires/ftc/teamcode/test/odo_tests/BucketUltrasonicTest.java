package org.firstinspires.ftc.teamcode.test.odo_tests;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;

@TeleOp(name = "Bucket Ultrasonic Test")
public class BucketUltrasonicTest extends LinearOpMode {

    private AnalogInput backLeftAngledUltrasonic;
    private AnalogInput backRightAngledUltrasonic;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        backLeftAngledUltrasonic =  hardwareMap.get(AnalogInput.class, HardwareMapNames.BACK_LEFT_SIDE_ULTRASONIC);
        backRightAngledUltrasonic =  hardwareMap.get(AnalogInput.class, HardwareMapNames.BACK_RIGHT_SIDE_ULTRASONIC);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, HardwareMapNames.PINPOINT);
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();

        waitForStart();

        while(opModeIsActive()) {
            pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);

            telemetry.addData("BACK LEFT VOLTAGE", backLeftAngledUltrasonic.getVoltage());
            telemetry.addData("BACK RIGHT VOLTAGE", backRightAngledUltrasonic.getVoltage());

            /*

            telemetry.addData("BACK LEFT", getDistanceFromVoltage(backLeftAngledUltrasonic.getVoltage()));
            telemetry.addData("BACK RIGHT", getDistanceFromVoltage(backRightAngledUltrasonic.getVoltage()));
            */

            /*
            Pose pose = new Pose(
                    getDistanceFromVoltage(backRightAngledUltrasonic.getVoltage()) * Math.sin(Math.PI + pinpoint.getHeading()),
                    getDistanceFromVoltage(backLeftAngledUltrasonic.getVoltage()) * Math.sin(Math.PI + pinpoint.getHeading()),
                    pinpoint.getHeading());

            Drawing.drawRobot(pose, "#4CAF50");
            Drawing.sendPacket();

            telemetry.addData("x", pose.getX());
            telemetry.addData("y", pose.getY());
            telemetry.addData("heading", Math.toDegrees(pose.getHeading()));
             */

            telemetry.update();
        }
    }

    private static double getDistanceFromVoltage(double voltage) {
        return 3300 * 520 / voltage;
    }
}
