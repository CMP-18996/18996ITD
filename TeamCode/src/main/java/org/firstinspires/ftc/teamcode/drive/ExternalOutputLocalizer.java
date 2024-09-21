package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.common.hardware.OpticalOdometer;
import org.firstinspires.ftc.teamcode.drive.util.Localizer;

@Config
public final class ExternalOutputLocalizer implements Localizer {
    private int lastPos;
    private OpticalOdometer optOdo;
    private boolean initialized = false;

    public ExternalOutputLocalizer(OpticalOdometer optOdo) {
        this.optOdo = optOdo;
        //FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
    }
    //ALL UNITS IN INCHES OR WHATEVER IT IS IDC
    public Twist2dDual<Time> update() {
        //FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));
        SparkFunOTOS.Pose2D currentPos = optOdo.getOtos().;
        SparkFunOTOS.Pose2D currentVelo = optOdo.getOtos().getVelocity();

        if (!initialized) {
            initialized = true;

            lastPos = currentPos;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int PosDelta = currentPos - lastPos;

        //std twist structure: fwd/back, strafe, angle
        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                1 /* this is the relative position change in fwd/back */,
                                1 /* this is the relative signed velocity in fwd/back */,
                        }),
                        new DualNum<Time>(new double[] {
                                1 /* this is the relative position change in strafe */,
                                1 /* this is the relative signed velocity in strafe */,
                        })
                ),
                new DualNum<>(new double[] {
                        1 /* this is the angle change, counterclockwise positive */,
                        1 /* this is the angular velocity */,
                })
        );
        //the reason i can't just take the global values the optical thingy spits out is bc i need a twist for everything to work
        //i could look into undoing the twist but that would take longer than i have now
        lastPos = currentPos;

        return twist;
    }
}
