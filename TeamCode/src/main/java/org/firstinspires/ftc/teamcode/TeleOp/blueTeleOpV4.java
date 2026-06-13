package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.sql.ResultSet;
import java.util.function.Supplier;

@TeleOp(name="blueTeleopV4")
public class blueTeleOpV4 extends OpMode {
    private int D;
    private final Pose startPose = new Pose(56, 30, Math.toRadians(90));
    private ElapsedTime SleepTimer = new ElapsedTime();
    private DcMotorEx deposit;
    private DcMotor intake;
    private static Follower follower;
    private ShooterCalculatons shooterCalculatons;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private double slowModeMultiplier = 0.5;
    private boolean automatedDrive = false;

    private boolean autoShooting = false;
    public double RPM = 0;
    public double error  =0;
    private Limelight3A limelight;

    // Four mecanum motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private static final double LIMELIGHT_MOUNT_ANGLE_DEG = 25.0;

    // Height of Limelight lens from the floor (inches)
    private static final double LIMELIGHT_LENS_HEIGHT_IN = 10;

    // Height of the target from the floor (inches)
    private static final double TARGET_HEIGHT_IN = 2.5;

    // Reject tiny blobs
    private static final double MIN_TA = 0.5;

    // How close tx must be to zero before we stop turning
    private static final double TX_TOLERANCE_DEG = 1.;

    // Proportional turning gain
    private static final double TURN_KP = 0.02;

    // Safety cap on turning power
    private static final double MAX_TURN_POWER = 0.35;
    boolean autograb = false;


    @Override

    public void init() {




        shooterCalculatons = new ShooterCalculatons();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 30, Math.toRadians(90)));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = hardwareMap.get(DcMotor.class, "intake"); // EH Port 0
        deposit = hardwareMap.get(DcMotorEx.class, "deposit"); // EH port 1
        deposit.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        deposit.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        deposit.setDirection(DcMotorSimple.Direction.REVERSE);
        // Tuned vals for P and F
        final double P = 65;
        final double F = 16.8;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        deposit.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Force pipeline 1
        limelight.pipelineSwitch(1);

        telemetry.addLine("Limelight Auto Turn + Distance Ready");
        telemetry.update();
    }


    @Override

    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        limelight.start();
    }

    @Override
    public void loop() {
        SleepTimer.reset();
        //Call this once per loop
        follower.update();
        if (!automatedDrive){
            //Make the last parameter false for field-centric
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric

            );}
        if (gamepad1.yWasPressed()) {
            follower.setStartingPose(new Pose(134, 8, Math.toRadians(90)));
        }

        // When gamepad-1 right bumper is pressed run intake 1 and 2 motors
        if (gamepad1.left_bumper) {
            intake.setPower(0.8);
        } else if (gamepad1.right_bumper) { // When Gamepad-1 left bumper is pressed reverse the intake motor
            intake.setPower(-0.8);
        } else { // if not stop the motor
            intake.setPower(0.0);
        }
        //Automated PathFollowing

        if (gamepad1.xWasPressed()) {
            RPM = -1000;
            autoShooting = false;

        }

        if (gamepad1.a) {
            double angle = shooterCalculatons.getthetablue(follower.getPose().getX(), follower.getPose().getY());
            error = angle - Math.toDegrees(follower.getPose().getHeading());
            if (Math.abs(error) < 2) {

            }
            else if (error < 0 ) {
                follower.setTeleOpDrive(0,0,-0.3);
            }
            else if(error > 0 ) {
                follower.setTeleOpDrive(0,0,0.3);
            }
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (!gamepad1.a)) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
        // When gamepad-2 right trigger is pressed start deposit motor.
        if (gamepad1.right_trigger > 0.1) {
            autoShooting  = true;
        } else if(gamepad1.left_trigger > 0.1) { // else stop the motor
            autoShooting = false;
            RPM = 0;
        }
        if (autoShooting){
            RPM=shooterCalculatons.autoshoot(follower.getPose().getX(),follower.getPose().getY(),false)-20;
        }

        double ticks = shooterCalculatons.rotationsToTicks(RPM);
        deposit.setVelocity(ticks);
        final double P = 65;
        final double F = 16.8;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        deposit.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);



        //deposit.setVelocity(getRPM(getDistance(), 35));
        //telemetry.addData("Deposit Servo Position", servoDeposit.getPosition());
        //telemetry.addData("Gate Servo Position", servoIntake.getPosition());
        //telemetry.addData("Intake Power Left", intake_2.getPower());
        //telemetry.addData("Intake Power Right", intake_3.getPower());
        telemetry.addData("Deposit Power", deposit.getPower());
        telemetry.addData("Current X pos", follower.getPose().getX());
        telemetry.addData("Current Y Pos", follower.getPose().getY());
        telemetry.addData("Current Heading", follower.getPose().getHeading());
        telemetry.addData("targetRPM",RPM);
        telemetry.addData("actualRPM",shooterCalculatons.ticksToRotations(deposit.getVelocity()));
        telemetry.addData("Distance",getDistance());
        telemetry.addData("Current RPM", getRPM(getDistance(), 35));
        telemetry.addData("blue angle", shooterCalculatons.getthetablue(follower.getPose().getX(), follower.getPose().getY()));
        telemetry.addData("red angle", shooterCalculatons.getthetared(follower.getPose().getX(), follower.getPose().getY()));
        telemetry.addData("error", error);

        if (gamepad1.dpad_up){
            autograb = true;
        } else {
            autograb = false;
        }

        if (gamepad1.dpad_right){
            limelight.pipelineSwitch(1);
        }
        else if (gamepad1.dpad_left){
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



public double getDistance()
    {
        Pose pose = follower.getPose();
        double targetX = 12; // 132
        double targetY = 132; // 132
        double distanceInches = Math.sqrt(Math.pow((pose.getX() - targetX), 2) + Math.pow(pose.getY() - targetY, 2));

        // Converting the distance from inches to mm by multiplying by 25.4.
        return distanceInches*2.54;
    }


    public double getRPM(double distance, double theta){

        // Gravity in mm
        double gravityMM = 980.694;
        double targetHeight = 81.28; // 2.54 * (43 - 11 )
        double thetaRadian = Math.toRadians(theta);

        // G is Gravity in mm 9806.94
        // d is horizontal distance from the goal to robot
        // theta is the hood angle in degrees
        // thetaRadian is the hood angle in radians
        // targetHight is the height of the goal in mm

        // v^2 = G*d^2/(2.cos(theta)^2(d.tan(theta)-targetHight)
        double numerator = (gravityMM*distance*distance);
        double denominator = (2* Math.pow(Math.cos(thetaRadian), 2)*(distance*Math.tan(thetaRadian)-targetHeight));

        telemetry.addData("target Height", targetHeight);
        telemetry.addData("Numerator", numerator);
        telemetry.addData("Denominator", denominator);

        if (denominator <=0) return 0;

        double velocity = Math.sqrt( numerator / denominator);

        telemetry.addData("velocity", velocity);

        double flywheelRadius = 4.8;
        return (60*velocity) / (2*Math.PI*flywheelRadius);

    }

}

