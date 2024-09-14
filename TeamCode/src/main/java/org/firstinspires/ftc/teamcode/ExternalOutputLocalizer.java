package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.messages.ThreeDeadWheelInputsMessage;

@Config
public final class ExternalOutputLocalizer implements Localizer {
    private int lastPos;
    private boolean initialized = false;

    public ExternalOutputLocalizer(HardwareMap hardwareMap) {
        //get from hardwaremap the thing u use for the optical sensor

        //FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
    }
    //ALL UNITS IN INCHES OR WHATEVER IT IS IDC
    public Twist2dDual<Time> update() {
        //FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));
        int currentPos = 1 /* get your current position from the optical */;
        //u probably will need seperate of these for x, y, theta or could bundle it up somehow

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
