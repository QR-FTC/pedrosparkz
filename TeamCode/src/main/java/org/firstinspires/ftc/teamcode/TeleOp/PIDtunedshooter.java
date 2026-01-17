package org.firstinspires.ftc.teamcode.TeleOp;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

    @TeleOp(name="tuningshooter")
    public class PIDtunedshooter extends OpMode {
        private ShooterCalculatons shooterCalculations;
        private Follower follower;
        Pose startpose = new Pose (72, 72, Math.toRadians(90));
        private DcMotorEx shooter;
        public double P = 65;
        public double I =0;
        public double D =0;
        public double F =16.8;
        public double RPM = 0;


        @Override
        public void init() {
            shooter = hardwareMap.get(DcMotorEx.class, "deposit");
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(startpose);
            shooterCalculations = new ShooterCalculatons();

        }

        @Override

        public void start() {
            follower.startTeleopDrive();
            //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
            //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
            //If you don't pass anything in, it uses the default (false)

        }

        @Override
        public void loop() {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            if(gamepad1.dpadDownWasPressed()) {
                RPM+=100;
            }
            if(gamepad1.dpadUpWasPressed()) {
                RPM -= 100;
            }
            if(gamepad1.dpadLeftWasPressed()) {
                RPM += 1000;
            }
            if(gamepad1.dpadRightWasPressed()) {
                RPM -= 1000;
            }
            if (gamepad1.xWasPressed()) {
                F = F+1;

            }
            if (gamepad1.yWasPressed()) {
                F = F-1;
            }
            if(gamepad1.aWasPressed()) {
                F = F + 0.1;
            }
            if (gamepad1.bWasPressed()) {
                F-=0.1;
            }
            if (gamepad1.rightBumperWasPressed()) {
                F/=10;
            }
            if(gamepad1.leftBumperWasPressed()) {
                F*=10;
            }

            double ticksec = shooterCalculations.rotationsToTicks(RPM);
            shooter.setVelocity(ticksec);
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
            ;

            // When gamepad-1 right bumper is pressed run intake 1 and 2 motors


            //deposit.setVelocity(getRPM(getDistance(), 35));

            //telemetry.addData("Deposit Servo Position", servoDeposit.getPosition());
            //telemetry.addData("Gate Servo Position", servoIntake.getPosition());
            //telemetry.addData("Intake Power Left", intake_2.getPower());
            //telemetry.addData("Intake Power Right", intake_3.getPower());
            follower.update();
            telemetry.addData("RPM", RPM);
            telemetry.addData("shooter velocity", shooterCalculations.ticksToRotations(shooter.getVelocity()));
            telemetry.addData("F", F);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("distance", shooterCalculations.distanceFromRed(follower.getPose().getX(), follower.getPose().getY()));
            telemetry.update();

        }
        public double getthetared(double x1, double y1) {
             return Math.atan2(144-x1, 144-y1);
        }
        public double getthetablue(double x1, double y1) {
            return Math.atan2(0-x1, 144-y1);
        }
}
