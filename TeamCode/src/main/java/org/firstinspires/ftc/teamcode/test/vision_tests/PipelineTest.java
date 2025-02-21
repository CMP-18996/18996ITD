package org.firstinspires.ftc.teamcode.test.vision_tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.common.vision.Color;
import org.firstinspires.ftc.teamcode.common.vision.ColorPipeline;
import org.firstinspires.ftc.teamcode.common.vision.ColorPipelineBuilder;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
@Config
public class PipelineTest extends CommandOpMode {
    ColorPipeline colorPipeline;
    VisionPortal visionPortal;
    public static double minSize = 1000;
    public static double maxSize = 100000;

    @Override
    public void initialize() {
        colorPipeline = ColorPipelineBuilder.createBuilder(Color.BLUE)
                .setMinSize(minSize)
                .setMaxSize(maxSize)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(CameraName.class, "Webcam 1"))
                .addProcessor(colorPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();
    }

    @Override
    public void run() {
        if (colorPipeline.specimenDetected()) {
            telemetry.addData("Specimen Heading:", colorPipeline.getAngle());
        }
        else {
            telemetry.addLine("No Specimen Detected");
        }
        telemetry.update();
        sleep(50);
    }
}
