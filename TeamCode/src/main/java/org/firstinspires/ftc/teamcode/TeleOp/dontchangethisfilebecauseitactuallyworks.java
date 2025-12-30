package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Limelight Distance + Angle (Vision Only)", group = "Vision")
public class dontchangethisfilebecauseitactuallyworks extends OpMode {

    // ================= HARDWARE =================
    private Limelight3A limelight;
    private IMU imu;

    // ================== TUNE THESE ==================

    // Measured Limelight mount angle (degrees)
    private static final double LIMELIGHT_MOUNT_ANGLE_DEG = 25.0;

    // Height of Limelight lens from floor (inches)
    private static final double LIMELIGHT_LENS_HEIGHT_IN = 7.5;

    // Height of target from floor (inches)
    private static final double TARGET_HEIGHT_IN = 2.5;

    // Reject very small blobs (noise)
    private static final double MIN_TA = 0.5;

    // =================================================

    @Override
    public void init() {

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // IMU (used only to keep Limelight pose math happy)
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(params);

        // Force Limelight pipeline 1
        limelight.pipelineSwitch(1);

        telemetry.addLine("Limelight Vision Only Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        // Start Limelight processing
        limelight.start();
    }

    @Override
    public void loop() {

        // Update yaw so Limelight math stays correct
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        // Get latest vision result
        LLResult llResult = limelight.getLatestResult();

        // ---------- VALIDITY CHECK ----------
        if (llResult == null || !llResult.isValid()) {
            telemetry.addLine("No valid target");
            telemetry.update();
            return;
        }

        // ---------- AREA FILTER ----------
        double ta = llResult.getTa();
        if (ta < MIN_TA) {
            telemetry.addLine("Target rejected (TA too small)");
            telemetry.addData("TA", ta);
            telemetry.update();
            return;
        }

        // ---------- ANGLES FROM CROSSHAIR ----------

        // Horizontal angle (left/right)
        double tx = llResult.getTx();

        // Vertical angle (up/down)
        double ty = llResult.getTy();

        // True vertical angle including camera tilt
        double trueVerticalAngleDeg =
                LIMELIGHT_MOUNT_ANGLE_DEG + ty;

        // Combined angular offset from crosshair
        double totalAngleOffsetDeg =
                Math.hypot(tx, ty);

        // ---------- DISTANCE CALCULATION ----------

        // Prevent invalid tan() values
        if (trueVerticalAngleDeg <= 1.0) {
            telemetry.addLine("Angle too small for distance math");
            telemetry.update();
            return;
        }

        double angleRad = Math.toRadians(trueVerticalAngleDeg);

        double distanceInches =
                (TARGET_HEIGHT_IN - LIMELIGHT_LENS_HEIGHT_IN)
                        / Math.tan(angleRad);

        // Force positive distance
        distanceInches = Math.abs(distanceInches);

        // ---------- TELEMETRY ----------
        telemetry.addData("tx (deg)", tx);
        telemetry.addData("ty (deg)", ty);
        telemetry.addData("True Vertical Angle (deg)", trueVerticalAngleDeg);
        telemetry.addData("Total Angle Offset (deg)", totalAngleOffsetDeg);
        telemetry.addData("TA", ta);
        telemetry.addData("Distance (in)", distanceInches);
        telemetry.addData("Distance (ft)", distanceInches / 12.0);
        telemetry.update();
    }
}
