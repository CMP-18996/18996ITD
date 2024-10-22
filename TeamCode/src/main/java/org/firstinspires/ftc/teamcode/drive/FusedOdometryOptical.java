package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.PoseVelocity2d;

public class FusedOdometryOptical {
    public static PoseVelocity2d fuse(PoseVelocity2d odo, PoseVelocity2d opt) {
        return new PoseVelocity2d(odo.linearVel.plus(opt.linearVel).div(2), 0.5 * (odo.angVel + opt.angVel));
    }
}
