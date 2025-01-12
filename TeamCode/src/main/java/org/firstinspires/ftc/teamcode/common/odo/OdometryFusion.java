package org.firstinspires.ftc.teamcode.common.odo;

import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.robot.OdometryHardware;

public class OdometryFusion
{

    private LocalizationMode localizationMode = LocalizationMode.NORMAL;
    private OdometryHardware odometryHardware;

    public enum LocalizationMode {
        NORMAL, // pinpoint, otos redundancy, limelight
        NORMAL_NO_VISION, // pinpoint, otos redundancy
        PINPOINT,
        OTOS,
        BUCKET_ALIGN, // 2x diagonal ultrasonics (WHEN DETECTED), pinpoint
        CHAMBER_ALIGN, // 1x back ultrasonic (WHEN DETECTED), pinpoint
        HUMAN_PLAYER_ALIGN // 1x side ultrasonic (WHEN DETECTED), pinpoint
    }

    public void update() {
        switch(localizationMode) {
            case NORMAL:

                break;
            case NORMAL_NO_VISION:

                break;
            case PINPOINT:

                break;
            case OTOS:

                break;
            case BUCKET_ALIGN:

                break;
            case CHAMBER_ALIGN:

                break;
            case HUMAN_PLAYER_ALIGN:

                break;
        }
    }

    public void reset() {
        // time dealy
    }

    public Pose2D getPosition() {
        return null;
    }

    public Pose2D getVelocity() {
        return null;
    }

    public void setPosition(Pose pos) {

    }

    public void setLocalizationMode(LocalizationMode localizationMode) {
        this.localizationMode = localizationMode;
    }
}
