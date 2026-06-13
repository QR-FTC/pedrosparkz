package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Limelight Auto Turn", group = "Vision")
public class balltracking extends OpMode {

    private Limelight3A limelight;
    private Follower follower;

    // ----- Tuning constants -----
    private static final double MIN_TA          = 0.5;   // ignore blobs smaller than this (noise)
    private static final double TX_TOLERANCE_DEG = 1.0;  // "close enough to centered" dead-band
    private static final double TURN_KP         = 0.02;  // proportional turn gain
    private static final double MAX_TURN_POWER  = 0.35;  // safety cap on turn power
    private static final double APPROACH_POWER  = 0.2;   // forward power while chasing a ball

    private boolean autograb = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        follower.update();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);   // force pipeline 1

        telemetry.addLine("Limelight Auto Turn Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Toggle auto-grab on/off with the d-pad
        if (gamepad1.dpadDownWasPressed()) {
            autograb = true;
        } else if (gamepad1.dpadUpWasPressed()) {
            autograb = false;
        }

        follower.update();

        if (autograb) {
            runAutoGrab();
        } else {
            // Normal driver control
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x);
        }

        telemetry.addData("Mode", autograb ? "AUTO-GRAB" : "DRIVER");
        telemetry.update();
    }
    
    private void runAutoGrab() {
        LLResult result = limelight.getLatestResult();

        // No usable target -> creep straight forward and bail
        if (result == null || !result.isValid() || result.getTa() < MIN_TA) {
            telemetry.addLine("No valid target");
            follower.setTeleOpDrive(APPROACH_POWER, 0, 0);
            return;
        }

        double tx = result.getTx();   // horizontal error: + = ball is to the right

        // Proportional turn, but zero inside the dead-band so we don't twitch
        double turnPower = 0;
        if (Math.abs(tx) > TX_TOLERANCE_DEG) {
            turnPower = TURN_KP * tx;
            turnPower = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, turnPower));
        }

        // Forward + steering at the same time
        follower.setTeleOpDrive(APPROACH_POWER, 0, turnPower);

        telemetry.addData("tx", tx);
        telemetry.addData("turnPower", turnPower);
    }
}