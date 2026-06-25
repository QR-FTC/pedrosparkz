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
import com.qualcomm.robotcore.hardware.Servo; // Changed from CRServo
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

    // --- SWERVE HARDWARE DEFINITIONS ---
    // Drive Motors
    private DcMotorEx lfDrive, rfDrive, lrDrive, rrDrive;

    // --- CHANGED TO REGULAR SERVOS ---
    private Servo lfTurn, rfTurn, lrTurn, rrTurn;

    // Robot Geometry Constants
    private final double L = 12.0;
    private final double W = 12.0;
    private final double R = Math.hypot(L, W);

    // --- HARDWARE ODOMETER ENCODER ---
    private DcMotorEx deadWheel;
    private final double TICKS_TO_INCHES = (1.26 * Math.PI) / 2000.0;

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

        // --- MAP SWERVE MOTORS ---
        lfDrive = hardwareMap.get(DcMotorEx.class, "lfDrive");
        rfDrive = hardwareMap.get(DcMotorEx.class, "rfDrive");
        lrDrive = hardwareMap.get(DcMotorEx.class, "lrDrive");
        rrDrive = hardwareMap.get(DcMotorEx.class, "rrDrive");

        // --- MAP STANDARD SERVOS ---
        lfTurn = hardwareMap.get(Servo.class, "lfTurn");
        rfTurn = hardwareMap.get(Servo.class, "rfTurn");
        lrTurn = hardwareMap.get(Servo.class, "lrTurn");
        rrTurn = hardwareMap.get(Servo.class, "rrTurn");

        // --- MAP THE DEAD-WHEEL ---
        deadWheel = hardwareMap.get(DcMotorEx.class, "par");
        deadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        follower.update();

        // 1. Capture joystick movements
        double stripeFwd = -gamepad1.left_stick_y;
        double stripeStrafe = gamepad1.left_stick_x;
        double rotationKey = gamepad1.right_stick_x;

        // 2. Swerve Kinematics Equations
        double A = stripeStrafe - rotationKey * (L / R);
        double B = stripeStrafe + rotationKey * (L / R);
        double C = stripeFwd - rotationKey * (W / R);
        double D = stripeFwd + rotationKey * (W / R);

        // Calculate Speeds
        double lfSpeed = Math.hypot(B, C);
        double rfSpeed = Math.hypot(B, D);
        double lrSpeed = Math.hypot(A, C);
        double rrSpeed = Math.hypot(A, D);

        // Calculate Target Steering Angles (Outputs range from -Math.PI to Math.PI)
        double lfAngle = Math.atan2(B, C);
        double rfAngle = Math.atan2(B, D);
        double lrAngle = Math.atan2(A, C);
        double rrAngle = Math.atan2(A, D);

        // Normalize speeds if any motor calculation exceeds 1.0
        double maxSpeed = Math.max(Math.max(lfSpeed, rfSpeed), Math.max(lrSpeed, rrSpeed));
        if (maxSpeed > 1.0) {
            lfSpeed /= maxSpeed; rfSpeed /= maxSpeed; lrSpeed /= maxSpeed; rrSpeed /= maxSpeed;
        }

        // 3. Assign Power Outputs to Drive Motors
        lfDrive.setPower(lfSpeed);
        rfDrive.setPower(rfSpeed);
        lrDrive.setPower(lrSpeed);
        rrDrive.setPower(rrSpeed);

        // --- CHANGED: CONVERT RADIANS TO SERVO POSITION (0.0 to 1.0) ---
        // Formula maps [-PI, PI] cleanly down to a [0.0, 1.0] fractional positional spectrum
        lfTurn.setPosition((lfAngle + Math.PI) / (2.0 * Math.PI));
        rfTurn.setPosition((rfAngle + Math.PI) / (2.0 * Math.PI));
        lrTurn.setPosition((lrAngle + Math.PI) / (2.0 * Math.PI));
        rrTurn.setPosition((rrAngle + Math.PI) / (2.0 * Math.PI));

        // --- PRESERVED DEAD-WHEEL TRACKING MATH ---
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double pitchRad = orientation.getPitch(AngleUnit.RADIANS);

        currentDistance = deadWheel.getCurrentPosition() * TICKS_TO_INCHES;
        double deltaWheel = currentDistance - lastDistance;
        double deltaHorizontal = deltaWheel * Math.cos(pitchRad);
        xHorizontal += deltaHorizontal;
        lastDistance = currentDistance;

        // Telemetry readout
        telemetryM.addData("Horizontal Odometer", xHorizontal);
        telemetryM.addData("LF Target Angle (Deg)", Math.toDegrees(lfAngle));
        telemetryM.addData("LF Servo Position (0-1)", lfTurn.getPosition());
        telemetryM.update();
    }
}