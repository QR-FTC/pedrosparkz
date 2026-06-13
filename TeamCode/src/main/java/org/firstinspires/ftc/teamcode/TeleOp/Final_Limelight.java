package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Limelight Distance + Angle (Vision Only)", group = "Vision")
public class Final_Limelight extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    // Measured Limelight mount angle (degrees)
    private static final double LIMELIGHT_MOUNT_ANGLE_DEG = 25.0;

    // Height of Limelight lens from floor (inches)
    private static final double LIMELIGHT_LENS_HEIGHT_IN = 7.5;

    // Height of target from floor (inches)
    private static final double TARGET_HEIGHT_IN = 2.5;

    // Reject very small blobs (noise)
    private static final double MIN_TA = 0.5;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(params);

        limelight.pipelineSwitch(1);

        telemetry.addLine("Limelight Vision Only Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult llResult = limelight.getLatestResult();

        if (llResult == null || !llResult.isValid()) {
            telemetry.addLine("No valid target");
            telemetry.update();
            return;
        }

        double ta = llResult.getTa();
        if (ta < MIN_TA) {
            telemetry.addLine("Target rejected (TA too small)");
            telemetry.addData("TA", ta);
            telemetry.update();
            return;
        }

        double tx = llResult.getTx();

        double ty = llResult.getTy();

        double trueVerticalAngleDeg =
                LIMELIGHT_MOUNT_ANGLE_DEG + ty;

        double totalAngleOffsetDeg =
                Math.hypot(tx, ty);

        if (trueVerticalAngleDeg <= 1.0) {
            telemetry.addLine("Angle too small for distance math");
            telemetry.update();
            return;
        }

        double angleRad = Math.toRadians(trueVerticalAngleDeg);

        double distanceInches =
                (TARGET_HEIGHT_IN - LIMELIGHT_LENS_HEIGHT_IN)
                        / Math.tan(angleRad);

        distanceInches = Math.abs(distanceInches);

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
