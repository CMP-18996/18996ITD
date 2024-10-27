package org.firstinspires.ftc.teamcode.test.miscellaneous;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name="LimelightInitial")
public class LimelightInitial extends LinearOpMode {
    private Limelight3A limelight;

    // For future reference limelight field of vision is 54 degrees horizontal, 41 degrees vertical
    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(11);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult latest = limelight.getLatestResult();
            if (latest != null && latest.isValid()) {
                Pose3D pose = latest.getBotpose();
                telemetry.addData("Tx", latest.getTx());
                telemetry.addData("Ty", latest.getTy());
                telemetry.addData("Bot pose", pose.toString());

                telemetry.update();
            }
            else {
                telemetry.addLine("Nothing Seen");
            }
            telemetry.update();
        }
        limelight.stop();
    }
}
