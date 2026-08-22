package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Angle Test", group = "Vision")
public class limelightonlytesting extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);

        telemetry.addLine("Limelight initialized");
        telemetry.addLine("Hold gamepad A to read target angle");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        if (!gamepad1.a) {
            telemetry.addLine("Hold A to detect pollen");
            telemetry.update();
            return;
        }

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            telemetry.addLine("No pollen detected");
            telemetry.update();
            return;
        }

        double angle = result.getTx();

        telemetry.addData("Pollen detected", "Yes");
        telemetry.addData("Horizontal angle", "%.1f degrees", angle);

        if (angle > 1.0) {
            telemetry.addData("Direction", "RIGHT");
        } else if (angle < -1.0) {
            telemetry.addData("Direction", "LEFT");
        } else {
            telemetry.addData("Direction", "CENTERED");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}