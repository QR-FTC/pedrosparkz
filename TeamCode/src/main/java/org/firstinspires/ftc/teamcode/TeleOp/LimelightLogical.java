package org.firstinspires.ftc.teamcode.TeleOp;

import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.ActiveOpMode;

public class LimelightLogical extends Command {
    public LimelightLogical() {
        requires(LimelightPhysical.INSTANCE);
    }
    @Override
    public boolean isDone() {
        return false;
    }
    @Override
    public void update() {
        LimelightPhysical ll = LimelightPhysical.INSTANCE;
        if (!ll.hasCluster()) {
            ActiveOpMode.INSTANCE.telemetry().addLine("No cluster seen ... haha bum");
        } else {
            ActiveOpMode.INSTANCE.telemetry().addData("Pollen in view", ll.getTotalBalls());
            ActiveOpMode.INSTANCE.telemetry().addData("largest cluster ball count", ll.getClusterSize());
            ActiveOpMode.INSTANCE.telemetry().addData("Cluster angle from center", "%.1f deg", ll.getClusterTx());
            ActiveOpMode.INSTANCE.telemetry().addData("Direction", ll.getDirection());
        }
        ActiveOpMode.INSTANCE.telemetry().update();
    }
}