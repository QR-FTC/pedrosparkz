package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "BallDriveAutoPipelineCommitted", group = "Vision")
public class teleopAuton extends OpMode {
enum AutoState {
    IDLE,
    FOLLOWING
}
AutoState autoState = AutoState.IDLE;
boolean lastA = false;
    // ================= HARDWARE =================
    private Limelight3A limelight;
    private static Follower follower;
    public static Pose ballpose;

    Pose robopose = follower.getPose();
    private final Pose robopose2 = new Pose(robopose.getX(), robopose.getY(), Math.toRadians(robopose.getHeading()));


    private IMU imu;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ================= LIMELIGHT =================
    private static final int PIPELINE_COUNT = 3;
    private static final double MIN_TA = 1.0;

    // ================= GEOMETRY =================
    private static final double MOUNT_ANGLE = 25.0;
    private static final double LENS_HEIGHT = 7.5;
    private static final double BALL_HEIGHT = 2.5;

    // ================= FORWARD CONTROL =================
    private static final double BASE_FORWARD = 0.18;   // always moving
    private static final double DIST_KP = 0.025;
    private static final double MAX_DRIVE = 0.45;

    // Slight intentional overshoot
    private static final double TARGET_DISTANCE = 6.5;

    // ================= TURN PD CONTROL =================
    private static final double TURN_KP = 0.010;
    private static final double TURN_KD = 0.003;
    private static final double MAX_TURN = 0.25;

    // ================= STATE =================
    private int lockedPipeline = -1;
    private double lastTx = 0;

    @Override
    public void init() {

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        telemetry.addLine("Committed Vision Drive Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        boolean aPressed = gamepad1.a;

        // Send yaw to Limelight
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult result = null;


        // ================= PIPELINE LOCK =================
        if (lockedPipeline == -1) {

            double bestTa = MIN_TA;

            for (int i = 0; i < PIPELINE_COUNT; i++) {
                limelight.pipelineSwitch(i);
                LLResult r = limelight.getLatestResult();

                if (r != null && r.isValid() && r.getTa() > bestTa) {
                    bestTa = r.getTa();
                    lockedPipeline = i;
                    result = r;
                }
            }




            if (lockedPipeline == -1) {
                setDrivePower(0, 0);
                telemetry.addLine("No target");
                telemetry.update();
                return;
            }

        } else {
            limelight.pipelineSwitch(lockedPipeline);
            result = limelight.getLatestResult();

            if (result == null || !result.isValid() || result.getTa() < MIN_TA) {
                lockedPipeline = -1;
                setDrivePower(0, 0);
                telemetry.addLine("Target lost");
                telemetry.update();
                return;
            }
        }

        // ================= TARGET DATA =================
        double tx = result.getTx();
        double ty = result.getTy();

        double trueAngle = MOUNT_ANGLE + ty;
        if (trueAngle < 2.0) {
            setDrivePower(0, 0);
            return;
        }

        double distance =
                Math.abs((BALL_HEIGHT - LENS_HEIGHT)
                        / Math.tan(Math.toRadians(trueAngle)));

        // ================= TURN PD =================
        double dTx = tx - lastTx;
        lastTx = tx;

        double turn = -(TURN_KP * tx + TURN_KD * dTx);
        turn = clamp(turn, -MAX_TURN, MAX_TURN);

        // Fade turn as target centers
        turn *= Math.min(Math.abs(tx) / 10.0, 1.0);

        // ================= FORWARD =================
        double distanceError = TARGET_DISTANCE - distance;

        // Always move forward
        double forward = BASE_FORWARD + distanceError * DIST_KP;
        forward = clamp(forward, 0.12, MAX_DRIVE);

        // ================= APPLY =================
        setDrivePower(forward, turn);

        telemetry.addData("Pipeline", lockedPipeline);
        telemetry.addData("tx", tx);
        telemetry.addData("Distance", distance);
        telemetry.addData("Forward", forward);
        telemetry.addData("Turn", turn);
        telemetry.update();
    }

    // ================= HELPERS =================
    private void setDrivePower(double forward, double turn) {
        frontLeft.setPower(forward + turn);
        backLeft.setPower(forward + turn);
        frontRight.setPower(forward - turn);
        backRight.setPower(forward - turn);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}