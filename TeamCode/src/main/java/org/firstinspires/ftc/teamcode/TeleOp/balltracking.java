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
    private double txToleranceDeg = 1.0;
    private double turnKp = 0.02;
    private double maxTurnPower = 0.35;
    private double clusterGapDeg = 8.0;
    private double scanTurnPower = 0.3;
    private double headingKp = 0.6;
    private double headingTolRad = Math.toRadians(4);
    private int lostFramesMax = 10;
    private int pipeline = 3;
    private enum State { SCANNING, GOTO_BEST, CHASING }
    private State state = State.SCANNING;
    private double lastHeading;
    private double accumulatedRotation;
    private int scanBestCount;
    private double scanBestHeading;
    private boolean scanHasBest;
    private int lostFrames;
    private boolean autograb = false;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        follower.update();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
        telemetry.addLine("Limelight Largest Cluster Tracker Ready");
        telemetry.addData("Pipeline", pipeline);
        telemetry.update();
    }
    @Override
    public void start() {
        limelight.start();
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        if (gamepad1.dpadDownWasPressed()) {
            autograb = true;
            startScanning();
        } else if (gamepad1.dpadUpWasPressed()) {
            autograb = false;
        }
        follower.update();
        telemetry.addData("Mode", autograb ? "AUTO-GRAB" : "DRIVER");
        if (autograb) {
            runAutoGrab();
        } else {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            telemetry.addLine("Driver control (d-pad down = auto-grab)");
        }
        telemetry.update();
    }
    private void startScanning() {
        state = State.SCANNING;
        lastHeading = follower.getPose().getHeading();
        accumulatedRotation = 0;
        scanBestCount = 0;
        scanHasBest = false;
        lostFrames = 0;
    }
    private void runAutoGrab() {
        ClusterInfo cluster = findLargestCluster(limelight.getLatestResult());
        double heading = follower.getPose().getHeading();
        telemetry.addData("State", state);
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
        switch (state) {
            case SCANNING: {
                follower.setTeleOpDrive(0, 0, scanTurnPower);
                accumulatedRotation += Math.abs(angleWrap(heading - lastHeading));
                lastHeading = heading;
                if (cluster != null && cluster.count > scanBestCount) {
                    scanBestCount = cluster.count;
                    scanBestHeading = angleWrap(heading - Math.toRadians(cluster.tx));
                    scanHasBest = true;
                }
                telemetry.addData("Scan (deg)", "%.0f / 360", Math.toDegrees(accumulatedRotation));
                telemetry.addData("Best cluster size", scanBestCount);
                if (accumulatedRotation >= 2 * Math.PI) {
                    if (scanHasBest) {
                        state = State.GOTO_BEST;
                    } else {
                        accumulatedRotation = 0;
                    }
                }
                break;
            }
            case GOTO_BEST: {
                double error = angleWrap(scanBestHeading - heading);
                if (Math.abs(error) <= headingTolRad) {
                    follower.setTeleOpDrive(0, 0, 0);
                    state = State.CHASING;
                } else {
                    follower.setTeleOpDrive(0, 0, clamp(-headingKp * error, maxTurnPower));
                }
                telemetry.addData("Heading error (deg)", "%.1f", Math.toDegrees(error));
                break;
            }
            case CHASING: {
                if (cluster == null) {
                    lostFrames++;
                    follower.setTeleOpDrive(0, 0, 0);
                    telemetry.addData("Lost frames", lostFrames);
                    if (lostFrames > lostFramesMax) {
                        startScanning();
                    }
                    break;
                }
                lostFrames = 0;
                double turnPower = 0;
                boolean centered = Math.abs(cluster.tx) <= txToleranceDeg;
                if (!centered) {
                    turnPower = clamp(turnKp * cluster.tx, maxTurnPower);
                }
                follower.setTeleOpDrive(0.2, 0, turnPower);
                telemetry.addData("Balls in view", cluster.totalBalls);
                telemetry.addData("Cluster size", cluster.count);
                telemetry.addData("Cluster angle (tx)", "%.1f", cluster.tx);
                telemetry.addData("Aim", centered ? "CENTERED" : (cluster.tx > 0 ? "RIGHT" : "LEFT"));
                telemetry.addData("Turn power", "%.3f", turnPower);
                break;
            }
        }
    }
    private ClusterInfo findLargestCluster(LLResult result) {
        if (result == null || !result.isValid()) return null;
        List<Double> angles = new ArrayList<>();
        for (LLResultTypes.ColorResult ball : result.getColorResults()) {
            angles.add(ball.getTargetXDegrees());
        }
        if (angles.isEmpty()) return null;
        Collections.sort(angles);
        int bestStart = 0, bestCount = 1;
        int runStart = 0, runCount = 1;
        for (int i = 1; i < angles.size(); i++) {
            if (angles.get(i) - angles.get(i - 1) <= clusterGapDeg) {
                runCount++;
            } else {
                runCount = 1;
                runStart = i;
            }
            if (runCount > bestCount) {
                bestCount = runCount;
                bestStart = runStart;
            }
        }
        double sum = 0;
        for (int i = bestStart; i < bestStart + bestCount; i++) {
            sum += angles.get(i);
        }
        return new ClusterInfo(sum / bestCount, bestCount, angles.size());
    }
    private static class ClusterInfo {
        final double tx;
        final int count;
        final int totalBalls;
        ClusterInfo(double tx, int count, int totalBalls) {
            this.tx = tx;
            this.count = count;
            this.totalBalls = totalBalls;
        }
    }
    private double clamp(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }
    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}