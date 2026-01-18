// =====================================================
//                        IMPORTS
// =====================================================
package org.firstinspires.ftc.teamcode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Evan Limelight Auto Turn + Distance", group = "Vision")
public class evanLimelight extends OpMode {

    // =====================================================
    //                     HARDWARE
    // =====================================================

    // Limelight vision camera
    private Limelight3A limelight;

    // Four mecanum motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // =====================================================
    //                 VISION / GEOMETRY CONSTANTS
    // =====================================================

    // Angle the Limelight is tilted upward from horizontal (degrees)
    private static final double LIMELIGHT_MOUNT_ANGLE_DEG = 25.0;

    // Height of Limelight lens from the floor (inches)
    private static final double LIMELIGHT_LENS_HEIGHT_IN = 9.5;

    // Height of the target from the floor (inches)
    private static final double TARGET_HEIGHT_IN = 2.5;

    // =====================================================
    //                 FILTERING & TURNING
    // =====================================================

    // Reject tiny blobs
    private static final double MIN_TA = 0.5;

    // How close tx must be to zero before we stop turning
    private static final double TX_TOLERANCE_DEG = 1.;

    // Proportional turning gain
    private static final double TURN_KP = 0.02;

    // Safety cap on turning power
    private static final double MAX_TURN_POWER = 0.35;

    Follower follower;
    boolean autograb = false;

    // =====================================================
    //                       INIT
    // =====================================================

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72, Math.toRadians(90)));
        follower.update();

        // ---------------- LIMELIGHT ----------------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Force pipeline 1
        limelight.pipelineSwitch(1);

        telemetry.addLine("Limelight Auto Turn + Distance Ready");
        telemetry.update();
    }

    // =====================================================
    //                      START
    // =====================================================

    @Override
    public void start() {
        // Start Limelight processing
        limelight.start();
        follower.startTeleopDrive();
    }

    // =====================================================
    //                       LOOP
    // =====================================================

    @Override
    public void loop() {

        if (gamepad1.dpadDownWasPressed()){
            autograb = true;
        } else if (gamepad1.dpadUpWasPressed()) {
            autograb = false;
        }

        if (gamepad1.aWasPressed()){
            limelight.pipelineSwitch(1);
        }
        else {
            limelight.pipelineSwitch(0);
        }

        follower.update();
        if (autograb){
            // ---------------- UPDATE YAW ----------------
            setTurnPower(0);

            // ---------------- GET LIMELIGHT RESULT ----------------
            LLResult llResult = limelight.getLatestResult();

            if (llResult == null || !llResult.isValid()) {
                telemetry.addLine("No valid target");
                telemetry.update();
                return;
            }

            // ---------------- TARGET AREA FILTER ----------------
            double ta = llResult.getTa();
            if (ta < MIN_TA) {
                telemetry.addLine("Rejected: TA too small");
                telemetry.addData("TA", ta);
                telemetry.update();
                return;
            }

            //                ANGLES FROM CROSSHAIRf

            // Horizontal offset from crosshair (degrees)
            // + = target right, - = target left
            double tx = llResult.getTx();

            // Vertical offset from crosshair (degrees)
            double ty = llResult.getTy();

            // True vertical angle to target (camera tilt + ty)
            double trueVerticalAngleDeg =
                    LIMELIGHT_MOUNT_ANGLE_DEG + ty;

            // Combined angular offset (optional debug)
            double totalAngleOffsetDeg =
                    Math.hypot(tx, ty);

            //                 AUTO-TURN LOGIC

            if (Math.abs(tx) > TX_TOLERANCE_DEG) {

                // Proportional control: more error → more turn
                double turnPower = TURN_KP * tx;

                // Clamp power for safety
                turnPower = Math.max(-MAX_TURN_POWER,
                        Math.min(MAX_TURN_POWER, turnPower));

                setTurnPower(turnPower);
            }

            //               DISTANCE CALCULATION

            double distanceInches = Double.NaN;

            // Prevent invalid tan() math
            if (trueVerticalAngleDeg > 1.0) {

                double angleRad = Math.toRadians(trueVerticalAngleDeg);

                distanceInches =
                        (TARGET_HEIGHT_IN - LIMELIGHT_LENS_HEIGHT_IN)
                                / Math.tan(angleRad);

                // Force positive distance
                distanceInches = Math.abs(distanceInches);
            }}
        else {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x);
        }

        //                     TELEMETRY

        telemetry.update();
    }

    //                        HELPER

    private void setTurnPower(double power) {
        follower.setTeleOpDrive(-gamepad1.left_stick_y,0,-power);
    }
}
