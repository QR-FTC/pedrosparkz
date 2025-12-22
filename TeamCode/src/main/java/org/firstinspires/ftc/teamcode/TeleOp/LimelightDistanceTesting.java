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

    // Limelight camera
    private Limelight3A limelight;

    // IMU for robot orientation
    private IMU imu;

    // ================== TUNE THESE VALUES ==================

    // Angle the Limelight is tilted upward (degrees)
    private static final double LIMELIGHT_MOUNT_ANGLE_DEG = 0.0;

    // Height of the Limelight lens from the floor (inches)
    private static final double LIMELIGHT_LENS_HEIGHT_IN = 0.0;

    // Height of the target from the floor (inches)
    private static final double TARGET_HEIGHT_IN = 0.0;

    // =======================================================

    @Override
    public void init() {

        // Get Limelight from the hardware map
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Get IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");

        // REQUIRED: Initialize IMU orientation for FTC
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParams);

        limelight.pipelineSwitch(1);

        telemetry.addLine("FTC Limelight Distance Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        // Start vision processing
        limelight.start();
    }

    @Override
    public void loop() {

        // Get the robot's yaw from the IMU
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        // Send yaw to Limelight so pose math is correct
        limelight.updateRobotOrientation(orientation.getYaw());

        // Get latest Limelight vision result
        LLResult llResult = limelight.getLatestResult();

        // Check if Limelight sees a valid target
        if (llResult != null && llResult.isValid()) {

            // Vertical angle offset from Limelight (degrees)
            double ty = llResult.getTy();

            // Total angle to the target
            double angleToTargetDeg = LIMELIGHT_MOUNT_ANGLE_DEG + ty;

            // Convert degrees to radians
            double angleToTargetRad = Math.toRadians(angleToTargetDeg);

            // Distance calculation using tangent
            double distanceInches =
                    (TARGET_HEIGHT_IN - LIMELIGHT_LENS_HEIGHT_IN)
                            / Math.tan(angleToTargetRad);

            // Telemetry output
            telemetry.addData("Tx (deg)", llResult.getTx());
            telemetry.addData("Ty (deg)", ty);
            telemetry.addData("Angle (deg)", angleToTargetDeg);
            telemetry.addData("Distance (in)", distanceInches);
            telemetry.addData("Distance (ft)", distanceInches / 12.0);

        } else {
            telemetry.addLine("No target detected");
        }

        telemetry.update();
    }
}