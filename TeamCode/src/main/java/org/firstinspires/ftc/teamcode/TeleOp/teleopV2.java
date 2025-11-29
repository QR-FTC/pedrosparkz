package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@TeleOp(name="teleopV2")
public class teleopV2 extends OpMode {
    private Servo servoDeposit;
    private DcMotor geckoWheels;
    private DcMotor intake_2;
    private DcMotor deposit;
    private DcMotor intake_3;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    @Override
    public void init() {
        //hi
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
        servoDeposit = hardwareMap.get(Servo.class, "Servo_Deposit");
        intake_3 = hardwareMap.get(DcMotor.class, "intake_3");
        intake_2= hardwareMap.get(DcMotor.class, "Intake_2");
        //geckoWheels = hardwareMap.get(DcMotor.class, "Deposit");
        deposit = hardwareMap.get(DcMotor.class, "Deposit");
        //while (opModeIsActive()) {
        //  waitForStart();


    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        //Call this once per loop
        if(gamepad1.dpad_left){
            intake_2.setPower(-0.8);
            intake_3.setPower(-0.8);
        }
        else if(gamepad1.dpad_right){
            intake_2.setPower(0.8);
            intake_3.setPower(0.8);

        }

        else {
            intake_2.setPower(0.0);
            intake_3.setPower(0.0);


        }
        if(gamepad1.right_trigger>0.1){
            deposit.setPower(gamepad1.right_trigger);
    }
        else if(gamepad1.left_trigger>0.1) {
            deposit.setPower(-gamepad1.left_trigger);
        }
        else {
            deposit.setPower(0.0);

        }
        if(gamepad1.dpad_up){
            servoDeposit.setPosition(0.8);
        }
        else if(gamepad1.dpad_down){
            servoDeposit.setPosition(-0.8);
        }
        else {
            servoDeposit.setPosition(0.0);
        }


//       // if(gamepad2.right_trigger>0.1) {
//        //    geckoWheels.setPower(gamepad2.right_trigger);
//        }
//       //else if (gamepad2.left_trigger>0.1){
//            geckoWheels.setPower(-gamepad2.left_trigger);
//        }
//        else {
//            geckoWheels.setPower(0.0);
//
//        }
//
//
//        if (gamepad2.dpad_up){
//            servoDeposit.setPower(1);
//
//        }
//        if(gamepad2.dpad_left){
//            servoDeposit.setPower(0);
//        }
//        if(gamepad2.dpad_down){
//            servoDeposit.setPower(-1);
//        }
//        follower.update();
//        telemetryM.update();
//        if (!automatedDrive) {
//            //Make the last parameter false for field-centric
//            //In case the drivers want to use a "slowMode" you can scale the vectors
//            //This is the normal version to use in the TeleOp
//            if (!slowMode) follower.setTeleOpDrive(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x,
//                    -gamepad1.right_stick_x,
//                    true // Robot Centric
//            );
//            //This is how it looks with slowMode on
//        }
//
//
//        telemetryM.debug("position", follower.getPose());
//        telemetryM.debug("velocity", follower.getVelocity());
//        telemetryM.debug("automatedDrive", automatedDrive);
    }
}
