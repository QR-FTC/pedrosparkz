package org.firstinspires.ftc.teamcode.TeleOp;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.util.function.Supplier;

@TeleOp(name="humpProject")
public class humpProject extends OpMode {
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private static Follower follower;
    IMU imu;

    // --- HARDWARE ENCODER DEFINITION ---
    private DcMotorEx deadWheel;

    // Conversion Factor: (Wheel Diameter * PI) / Ticks Per Rev
    // (Example assumes a 32mm / 1.26" tracking wheel and a 2000 count encoder)
    private final double TICKS_TO_INCHES = (1.26 * Math.PI) / 2000.0;

    // These start at 0.0 on initialization
    private double currentDistance = 0.0;
    private double lastDistance = 0.0;
    private double xHorizontal = 0.0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(36, 12, 90));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // --- MAP THE ENCODER HERE ---
        // Replace "par" with whatever name you gave your tracking wheel encoder pin in your hardware configuration
        deadWheel = hardwareMap.get(DcMotorEx.class, "par");
        deadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double pitchRad = orientation.getPitch(AngleUnit.RADIANS);

        // --- OVERWRITE THE INITIAL 0.0 WITH THE REAL LIVE VALUE ---
        // Every frame, this pulls raw data from your tracking pod and changes currentDistance
        currentDistance = deadWheel.getCurrentPosition() * TICKS_TO_INCHES;

        // Now deltaWheel will not be 0.0 because currentDistance changes as the robot moves!
        double deltaWheel = currentDistance - lastDistance;
        double deltaHorizontal = deltaWheel * Math.cos(pitchRad);
        xHorizontal += deltaHorizontal;

        // Hand off the updated distance for the next frame calculation
        lastDistance = currentDistance;

        // Verify the numbers are fluctuating live on driver hub telemetry
        telemetryM.addData("Raw Encoder Ticks", deadWheel.getCurrentPosition());
        telemetryM.addData("Current Computed Distance", currentDistance);
        telemetryM.addData("X Horizontal Odometer", xHorizontal);
        telemetryM.update();
    }
}