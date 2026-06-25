package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@TeleOp(name = "Limelight Largest Cluster Tracker", group = "Vision")
public class balltracking extends OpMode {

    private Limelight3A limelight;
    private Follower follower;

    // ----- Tuning constants -----
    private static final double MIN_TA            = 0.5;  // ignore blobs smaller than this (noise)
    private static final double TX_TOLERANCE_DEG  = 1.0;  // "close enough to centered" dead-band
    private static final double TURN_KP           = 0.02; // proportional turn gain
    private static final double MAX_TURN_POWER    = 0.35; // safety cap on turn power
    private static final double APPROACH_POWER    = 0.2;  // forward power while chasing
    private static final double CLUSTER_GAP_DEG   = 10.0; // balls farther apart than this = different clusters

    private boolean autograb = false;
    
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        follower.update();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);   // force pipeline 1

        telemetry.addLine("Limelight Largest Cluster Tracker Ready");
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
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x);
        }

        telemetry.addData("Mode", autograb ? "AUTO-GRAB" : "DRIVER");
        telemetry.update();
    }

    /** Steers toward the center of the cluster that contains the most balls. */
    private void runAutoGrab() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addLine("No valid result");
            follower.setTeleOpDrive(APPROACH_POWER, 0, 0);
            return;
        }

        // 1) Collect the horizontal angle of every real ball, sorted left -> right
        List<Double> ballAngles = new ArrayList<>();
        for (LLResultTypes.ColorResult ball : result.getColorResults()) {
            if (ball.getTargetArea() >= MIN_TA) {
                ballAngles.add(ball.getTargetXDegrees()); // + = right of center
            }
        }
        Collections.sort(ballAngles);

        if (ballAngles.isEmpty()) {
            telemetry.addLine("No balls detected");
            follower.setTeleOpDrive(APPROACH_POWER, 0, 0);
            return;
        }

        // 2) Walk left-to-right; start a new cluster whenever there's a big gap
        int bestCount = 0, bestStart = 0;          // biggest cluster found so far
        int runCount  = 1, runStart  = 0;          // cluster we're currently building
        for (int i = 1; i < ballAngles.size(); i++) {
            if (ballAngles.get(i) - ballAngles.get(i - 1) <= CLUSTER_GAP_DEG) {
                runCount++;                         // same cluster
            } else {
                runCount = 1;                       // gap -> new cluster
                runStart = i;
            }
            if (runCount > bestCount) {
                bestCount = runCount;
                bestStart = runStart;
            }
        }

        // 3) Aim at the average angle of the balls in that biggest cluster
        double sum = 0;
        for (int i = bestStart; i < bestStart + bestCount; i++) {
            sum += ballAngles.get(i);
        }
        double clusterTx = sum / bestCount;

        // 4) Proportional turn toward the cluster, with dead-band + clamp
        double turnPower = 0;
        if (Math.abs(clusterTx) > TX_TOLERANCE_DEG) {
            turnPower = TURN_KP * clusterTx;
            turnPower = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, turnPower));
        }
        follower.setTeleOpDrive(APPROACH_POWER, 0, turnPower);

        telemetry.addData("Total balls", ballAngles.size());
        telemetry.addData("Largest cluster size", bestCount);
        telemetry.addData("Cluster angle (tx)", clusterTx);
        telemetry.addData("Turn power", turnPower);
    }
}