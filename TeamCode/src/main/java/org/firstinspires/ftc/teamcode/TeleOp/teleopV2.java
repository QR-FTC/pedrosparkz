package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@TeleOp(name="teleopV2")
public class teleopV2 extends OpMode {
    private int D;

    private Servo servoIntake;
    private Servo servoDeposit;
    private ElapsedTime SleepTimer = new ElapsedTime();
    private DcMotor geckoWheels;
    private DcMotor intake_2;
    private DcMotor deposit;
    private DcMotor intake_3;
    private static Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(36, 12, 90));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
        servoDeposit = hardwareMap.get(Servo.class, "servoDeposit");
        servoIntake = hardwareMap.get(Servo.class, "servoIntake");
        intake_3 = hardwareMap.get(DcMotor.class, "intake_3");
        intake_2 = hardwareMap.get(DcMotor.class, "intake_2");
        deposit = hardwareMap.get(DcMotor.class, "depositMotor");

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
        SleepTimer.reset();
        //Call this once per loop
        follower.update();
        Pose pose = follower.getPose();
        double t =2;
        double d=0.096;
        double x = pose.getX();
        double y = pose.getY();
        double targetX = 12;
        double targetY = 132;
        double BDistance = Math.sqrt(Math.pow((x - 12), 2) + Math.pow(y - 132, 2));
        double RDistance = Math.sqrt(Math.pow((x - 132), 2) + Math.pow(y - 132, 2));
        double RVelocity = ((2*RDistance)/(39.37*t));
        double BVelocity = ((2*BDistance)/(39.37*t));
        double BRPM = (60*BVelocity)/(Math.PI*d);
        double RRPM = (60*RVelocity) / (Math.PI*d);
        double r = RRPM/3400;
        double b = BRPM/3400;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );


            if (gamepad1.right_bumper) {
                intake_2.setPower(0.8);
                intake_3.setPower(-0.8);
            } else if (gamepad1.left_bumper) {
                intake_2.setPower(-0.8);
                intake_3.setPower(0.8);

            } else {

                intake_2.setPower(0.0);
                intake_3.setPower(0.0);
            }


            if (gamepad2.right_trigger > 0.1) {
                deposit.setPower(-0.75 * (gamepad2.right_trigger));
            } else {
                deposit.setPower(0.0);

            }

                if (gamepad2.dpad_up) {
                    servoDeposit.setPosition(0.8);
                    while (SleepTimer.milliseconds() < 150) {
                        telemetry.update();
                    }
                }
                if(gamepad1.y) {
                    deposit.setPower(r);
                    double power = getRPM(8800, 35);

                }
                if(gamepad1.x) {
                    deposit.setPower(b);
                }

//<<<<<
                         else if (gamepad2.dpad_down) {
//>>>>>>>5 a055f0a784bf2972d2dc0910d3fb1e26b27126d
                            servoDeposit.setPosition(0.3);
                            while (SleepTimer.milliseconds() < 150) {
                                telemetry.update();
                            }
                        }
//<<<<<<<HEAD

                        if (gamepad2.b) {
                            servoIntake.setPosition(0.1);
                            while (SleepTimer.milliseconds() < 150) {
                                telemetry.update();
                            }
                        } else if (gamepad2.a) {
                            servoIntake.setPosition(0.2);
                            while (SleepTimer.milliseconds() < 150) {
                                telemetry.update();
                            }
                        }
                        if (gamepad2.x) {
                            servoDeposit.setPosition(0.3);
                            servoIntake.setPosition(0.2);
                            while (SleepTimer.milliseconds() < 150) {
                                telemetry.update();
                            }
                        }
//=======



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
////        telemetryM.update();
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
//<<<<<<<HEAD
                                telemetry.addData("Deposit Servo Position", servoDeposit.getPosition());
                                telemetry.addData("Gate Servo Position", servoIntake.getPosition());
                                telemetry.addData("Intake Power Left", intake_2.getPower());
                                telemetry.addData("Intake Power Right", intake_3.getPower());
                                telemetry.addData("Deposit Power", deposit.getPower());
                                telemetry.addData("x", x);
                                telemetry.addData("y", y);
                                telemetry.addData("heading", follower.getPose().getHeading());
                                telemetry.addData("Distance to Blue Scoring", BDistance);
                                telemetry.addData("Distance to Red Scoring", RDistance);
                                telemetry.addData("Shoot to Red Velocity", RVelocity);
                                telemetry.addData("Shoot to Blue Velocity", BVelocity);
                                telemetry.update();
//=======

//
//        telemetryM.debug("position", follower.getPose());
//        telemetryM.debug("velocity", follower.getVelocity());
//        telemetryM.debug("automatedDrive", automatedDrive);

    }

    public double getRPM(double distance, double theta){

        double output = (9806.94*distance*distance) / (2* Math.pow(Math.cos(theta), 2)*(distance*Math.tan(theta)-812.8));
        output = Math.sqrt(output);
        double RPM = (60*output) / (2*Math.PI*96);
        return RPM;
    }
}
