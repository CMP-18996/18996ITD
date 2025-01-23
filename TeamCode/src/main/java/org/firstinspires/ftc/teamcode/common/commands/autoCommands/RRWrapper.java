package org.firstinspires.ftc.teamcode.common.commands.autoCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.drive.SparkFunOTOSDrive;

public class RRWrapper extends CommandBase {
    private final Action action;
    private boolean finished = false;
    SparkFunOTOSDrive drive;

    public RRWrapper(SparkFunOTOSDrive drive, Action action) {
        this.action = action;
        this.drive = drive;
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        action.preview(packet.fieldOverlay());
        finished = !action.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
