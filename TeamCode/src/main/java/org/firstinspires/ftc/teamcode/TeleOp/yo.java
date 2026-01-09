package org.firstinspires.ftc.teamcode.TeleOp;

// =====================================================
//                        IMPORTS
// =====================================================
// These imports give us access to FTC hardware, Limelight,
// IMU orientation data, and OpMode structure.

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "test", group = "Vision")
public class yo extends OpMode {

    // All physical devices on the robot

    // Limelight camera
    private Limelight3A limelight;

    // IMU used only to send yaw to Limelight (pose stability)
    private IMU imu;

    // Four mecanum drivetrain motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // These MUST be measured on your robot

    // Angle the camera is tilted UP from horizontal
    private static final double MountAngle = 25.0;

    // Height of Limelight lens above the floor
    private static final double Lensheight = 7.5;

    // Height of the ball center above the floor
    private static final double BallHeight = 2.5;

    // Ignore very small blobs (noise, reflections, random pixels)
    private static final double MinTa = 1.0;

    // Minimum power needed to overcome drivetrain friction
    private static final double MinTurnPower = 0.08;

    // Safety cap so we don't spin violently
    private static final double MaxTurnPower = 0.35;

    // Timer to prevent pipeline spam
    private long lastPipelineSwitchTime = 0;

    // Which pipeline we are currently scanning
    private int scanningPipeline = 0;

    // Last valid result seen on each pipeline
    private LLResult lastResult0 = null;
    private LLResult lastResult1 = null;

    // Target locking state
    private boolean targetLocked = false;
    private int activePipeline = -1;
    private int candidatePipeline = -1;
    private int stableFrameCount = 0;

    // Runs once when INIT is pressed

    @Override
    public void init() {

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse right side so forward power moves robot forward
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Start on pipeline 0
        limelight.pipelineSwitch(0);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        telemetry.addLine("Ball Tracking (Merged) Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        // Start Limelight processing
        limelight.start();
    }

    // Runs repeatedly while the OpMode is active

    @Override
    public void loop() {

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        // Default to stopped every loop
        setDrivePower(0, 0);

        // We alternate pipelines and remember the largest
        // target we see before locking onto it

        if (!targetLocked) {

            long now = System.currentTimeMillis();

            // Switch pipelines only if enough time has passed
            if (now - lastPipelineSwitchTime > 300) {
                scanningPipeline = (scanningPipeline == 0) ? 1 : 0;
                limelight.pipelineSwitch(scanningPipeline);
                lastPipelineSwitchTime = now;
            }

            // Get the latest Limelight frame
            LLResult current = limelight.getLatestResult();

            // Save valid detections for each pipeline
            if (current != null && current.isValid() && current.getTa() > MinTa) {
                if (scanningPipeline == 0) {
                    lastResult0 = current;
                } else {
                    lastResult1 = current;
                }
            }

            // Compare target sizes
            double ta0 = (lastResult0 != null) ? lastResult0.getTa() : 0;
            double ta1 = (lastResult1 != null) ? lastResult1.getTa() : 0;

            int bestPipeline = -1;

            // Choose the pipeline with the larger target
            if (ta0 > ta1 && ta0 > MinTa) bestPipeline = 0;
            else if (ta1 > ta0 && ta1 > MinTa) bestPipeline = 1;

            // Stability check before locking
            if (bestPipeline != -1) {
                if (bestPipeline == candidatePipeline) {
                    stableFrameCount++;
                } else {
                    candidatePipeline = bestPipeline;
                    stableFrameCount = 1;
                }

                // Lock after enough stable frames
                if (stableFrameCount >= 5) {
                    targetLocked = true;
                    activePipeline = candidatePipeline;
                    limelight.pipelineSwitch(activePipeline);
                }
            }

            telemetry.addData("Scanning Pipeline", scanningPipeline);
            telemetry.addData("TA Pipeline 0", ta0);
            telemetry.addData("TA Pipeline 1", ta1);
            telemetry.addData("Target Locked", targetLocked);
            telemetry.update();
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        // Horizontal and vertical offsets from crosshair
        double tx = result.getTx();
        double ty = result.getTy();

        // True vertical angle to the target
        double trueVerticalAngle =
                MountAngle + ty;

        // Distance calculation using trig
        double distanceInches = Math.abs(
                (BallHeight - Lensheight) /
                        Math.tan(Math.toRadians(trueVerticalAngle))
        );

        double turn = 0;
        // If the bot is in this range it is "alligned"
        if (Math.abs(tx) > 1) {
            turn = -0.015 * tx;

            // Ensure minimum power to actually move
            if (turn > 0) {
                turn = Math.max(turn, MinTurnPower);
            } else {
                turn = Math.min(turn, -MinTurnPower);
            }

            turn = clamp(turn, -MaxTurnPower, MaxTurnPower);
        }

        double forward = 0;
        double error = distanceInches - 8;

        // Drive only if we are too far away
        if (error > 0) {
            forward = Math.min(error * .03, .4);

            // Slow down near target
            if (error < 3) forward *= 0.5;
        }

        // Apply movement
        setDrivePower(forward, turn);

        telemetry.addData("Active Pipeline", activePipeline);
        telemetry.addData("tx", tx);
        telemetry.addData("Distance (in)", distanceInches);
        telemetry.addData("Forward Power", forward);
        telemetry.addData("Turn Power", turn);
        telemetry.update();
    }

    private void setDrivePower(double forward, double turn) {
        frontLeft.setPower(forward + turn);
        backLeft.setPower(forward + turn);
        frontRight.setPower(forward - turn);
        backRight.setPower(forward - turn);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}