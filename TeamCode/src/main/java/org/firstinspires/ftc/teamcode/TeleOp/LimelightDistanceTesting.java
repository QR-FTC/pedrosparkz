package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Pipeline Distance FTC", group = "Vision")
public class LimelightDistanceTesting extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    // ===== TUNE THESE =====
    private static final double LIMELIGHT_MOUNT_ANGLE_DEG = 25.0;
    private static final double LIMELIGHT_LENS_HEIGHT_IN = 20.0;
    private static final double TARGET_HEIGHT_IN = 60.0;

    // Minimum target area to accept (filters grains/noise)
    private static final double MIN_TA = 0.5;
    // =====================

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParams);

        // Force pipeline 1
        limelight.pipelineSwitch(1);

        telemetry.addLine("FTC Limelight Ready");
        telemetry.addLine("Pipeline: 1");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        // Update robot yaw for Limelight pose calculations
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {

            double ta = llResult.getTa();

            // ❌ Reject small blobs (noise / grains)
            if (ta < MIN_TA) {
                telemetry.addLine("Target rejected (TA too small)");
                telemetry.addData("TA", ta);
                telemetry.update();
                return;
            }

            double ty = llResult.getTy();

            double angleToTargetDeg = LIMELIGHT_MOUNT_ANGLE_DEG + ty;
            double angleToTargetRad = Math.toRadians(angleToTargetDeg);

            double distanceInches =
                    (TARGET_HEIGHT_IN - LIMELIGHT_LENS_HEIGHT_IN)
                            / Math.tan(angleToTargetRad);

            // ✅ Ensure distance is never negative
            distanceInches = Math.abs(distanceInches);

            telemetry.addData("Pipeline", 1);
            telemetry.addData("Tx (deg)", llResult.getTx());
            telemetry.addData("Ty (deg)", ty);
            telemetry.addData("TA", ta);
            telemetry.addData("Angle (deg)", angleToTargetDeg);
            telemetry.addData("Distance (in)", distanceInches);
            telemetry.addData("Distance (ft)", distanceInches / 12.0);

        } else {
            telemetry.addLine("No valid target");
        }

        telemetry.update();
    }
}