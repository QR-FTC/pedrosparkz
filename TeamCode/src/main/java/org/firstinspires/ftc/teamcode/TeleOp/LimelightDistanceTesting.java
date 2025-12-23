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

    // ⚠ SET REAL VALUES BEFORE MATCH
    private static final double LIMELIGHT_MOUNT_ANGLE_DEG = 25.0;
    private static final double LIMELIGHT_LENS_HEIGHT_IN = 7.5;
    private static final double TARGET_HEIGHT_IN = 2.5;

    private static final double MIN_TA = 0.5;
    private static final double MIN_ANGLE_DEG = 5.0;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);

        limelight.pipelineSwitch(0);

        telemetry.addLine("Limelight Distance Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        // Update yaw for Limelight
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult llResult = limelight.getLatestResult();

        telemetry.addLine("=== LIMELIGHT DEBUG ===");

        if (llResult == null || !llResult.isValid()) {
            telemetry.addLine("No valid target");
            telemetry.update();
            return;
        }

        double tx = llResult.getTx();
        double ty = llResult.getTy();
        double ta = llResult.getTa();

        telemetry.addData("Tx", tx);
        telemetry.addData("Ty", ty);
        telemetry.addData("TA", ta);

        // ----- TA FILTER -----
        if (ta < MIN_TA) {
            telemetry.addLine("Status: Rejected (TA too small)");
            telemetry.update();
            return;
        }

        double angleToTargetDeg = LIMELIGHT_MOUNT_ANGLE_DEG + ty;
        telemetry.addData("Angle (deg)", angleToTargetDeg);

        // ----- ANGLE FILTER -----
        if (Math.abs(angleToTargetDeg) < MIN_ANGLE_DEG) {
            telemetry.addLine("Status: Rejected (Angle too small)");
            telemetry.update();
            return;
        }

        double distanceInches =
                (TARGET_HEIGHT_IN - LIMELIGHT_LENS_HEIGHT_IN)
                        / Math.tan(Math.toRadians(angleToTargetDeg));

        telemetry.addLine("Status: VALID");
        telemetry.addData("Distance (in)", distanceInches);
        telemetry.addData("Distance (ft)", distanceInches / 12.0);

        telemetry.update();
    }
}
