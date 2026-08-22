package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "mimelighsted", group = "monke")
public class LimelightOpMode extends NextFTCOpMode {
    public LimelightOpMode() {
        addComponents(new SubsystemComponent(LimelightPhysical.INSTANCE));
    }
    @Override
    public void onStartButtonPressed() {
        new LimelightLogical().schedule();
    }
}