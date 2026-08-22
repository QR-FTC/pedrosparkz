package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class LimelightPhysical implements Subsystem {

    public static final LimelightPhysical INSTANCE = new LimelightPhysical();

    private LimelightPhysical() {}

    private Limelight3A limelight;
    private final int pipeline = 5;
    private final double clusterGapDeg = 8.0;
    private boolean hasCluster = false;
    private int totalBalls = 0;
    private int clusterSize = 0;
    private double clusterTx = 0.0;

    @Override
    public void initialize() {
        limelight = ActiveOpMode.INSTANCE.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    @Override
    public void periodic() {
        computeLargestCluster(limelight.getLatestResult());
    }

    public boolean hasCluster()   { return hasCluster; }
    public double  getClusterTx() { return clusterTx; }
    public int     getClusterSize() { return clusterSize; }
    public int     getTotalBalls()  { return totalBalls; }

    public String getDirection() {
        if (!hasCluster) return "NONE";
        if (clusterTx > 1.0) return "RIGHT";
        if (clusterTx < -1.0) return "LEFT";
        return "CENTERED";
    }

    private void computeLargestCluster(LLResult result) {
        if (result == null || !result.isValid()) { clear(); return; }

        List<Double> angles = collectBallAngles(result);
        if (angles.isEmpty()) { clear(); return; }

        int[] range = findBiggestClusterRange(angles);
        totalBalls = angles.size();
        clusterSize = range[1];
        clusterTx = averageCluster(angles, range[0], range[1]);
        hasCluster = true;
    }

    private List<Double> collectBallAngles(LLResult result) {
        List<Double> angles = new ArrayList<>();
        for (LLResultTypes.ColorResult ball : result.getColorResults()) {
            angles.add(ball.getTargetXDegrees());
        }
        Collections.sort(angles);
        return angles;
    }

    private int[] findBiggestClusterRange(List<Double> angles) {
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
        return new int[] { bestStart, bestCount };
    }

    private double averageCluster(List<Double> angles, int start, int count) {
        double sum = 0;
        for (int i = start; i < start + count; i++) {
            sum += angles.get(i);
        }
        return sum / count;
    }

    private void clear() {
        hasCluster = false;
        totalBalls = 0;
        clusterSize = 0;
        clusterTx = 0.0;
    }
}