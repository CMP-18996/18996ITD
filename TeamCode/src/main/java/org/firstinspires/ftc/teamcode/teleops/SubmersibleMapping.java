package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.robot.HardwareMapNames;
import org.firstinspires.ftc.teamcode.common.robot.PreMatchData;
import org.firstinspires.ftc.teamcode.common.robot.subsystems.IntakeSubsystem;

@TeleOp(name = "Submersible Mapping")
public class SubmersibleMapping extends LinearOpMode {
    private double x;
    private double y;
    private double h;

    GamepadEx gamepad;

    @Override
    public void runOpMode() {
        gamepad = new GamepadEx(gamepad1);

        waitForStart();

        while(opModeIsActive()) {

            if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                x += 1;
            }
            else if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                x -= 1;
            }
            else if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                y += 1;
            }
            else if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                y -= 1;
            }
            else if(gamepad.wasJustPressed(GamepadKeys.Button.Y)) {
                x += 0.1;
            }
            else if(gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                x -= 0.1;
            }
            else if(gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                y += 0.1;
            }
            else if(gamepad.wasJustPressed(GamepadKeys.Button.B)) {
                y -= 0.1;
            }
            else if(gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                h += 5;
            }
            else if(gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                h -= 5;
            }

            if(x < 0) {x = 0;}
            if(x > 50) {x = 50;}

            if(y < 0) {y = 0;}
            if(y > 75) {y = 75;}

            if(h < -90) {h = -90;}
            if(h > 90) {h = 90;}

            PreMatchData.sampleX = x / 2.54;
            PreMatchData.sampleY = y / 2.54;
            PreMatchData.sampleH = h / Math.toRadians(h);

            telemetry.addLine("COORDINATES FROM BOTTOM RIGHT SUB CORNER");
            telemetry.addLine("X IS FORWARD, Y IS LEFT-RIGHT");
            telemetry.addLine("VERTICAL IS 0 DEGREES, INCREASES COUNTERCLOCKWISE");

            telemetry.addData("X (CM)", x);
            telemetry.addData("Y FROM RIGHT (CM)", y);
            telemetry.addData("Y FROM LEFT (CM)", 75 - y); // 74.93 really
            telemetry.addData("H (DEGREES)", h);
            telemetry.update();
        }
    }
}

