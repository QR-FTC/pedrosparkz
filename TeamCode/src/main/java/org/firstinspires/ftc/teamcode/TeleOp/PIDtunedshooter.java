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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

    @TeleOp(name="tuningshooter")
    public class PIDtunedshooter extends OpMode {
        private ShooterCalculatons shooterCalculations;
        private final Pose startPose = new Pose(48, 25, Math.toRadians(90));

        private DcMotorEx shooter;
        private Servo Gateservo;

        private DcMotor intake;
        public double P = 61.3;
        public double I =0;
        public double D =0;
        public double F =15;
        public double RPM = 0;


        @Override
        public void init() {
            Gateservo = hardwareMap.get(Servo.class, "servo");

            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(startPose);
            follower.update();
            shooter = hardwareMap.get(DcMotorEx.class, "deposit");
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
            shooterCalculations = new ShooterCalculatons();
            intake = hardwareMap.get(DcMotor.class, "intake"); // EH Port 0


        }

        @Override

        public void start() {
            //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
            //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
            //If you don't pass anything in, it uses the default (false)

        }

        @Override
        public void loop() {
            if(gamepad1.dpadDownWasPressed()) {
                RPM+= 100;
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
                F= F + 0.1;
            }
            if (gamepad1.bWasPressed()) {
                F-=0.1;
            }
            if (gamepad2.bWasPressed()) {
                F/=10;
            }
            if(gamepad2.aWasPressed()) {
                F*=10;
            }
            if (gamepad2.left_bumper) {
                Gateservo.setPosition(0.1);
            }
            if (gamepad2.right_bumper) {
                Gateservo.setPosition(0.35);
            }

            if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else if (gamepad1.right_bumper) { // When Gamepad-1 left bumper is pressed reverse the intake motor
                intake.setPower(1);
            } else { // if not stop the motor
                intake.setPower(0.0);
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
            telemetry.addData("target RPM", RPM);
            telemetry.addData("P", P);
            telemetry.addData("F", F);
            telemetry.addData("actual RPM" , shooterCalculations.ticksToRotations(shooter.getVelocity()));
            telemetry.addData("distance", shooterCalculations.distanceFromBlue(follower.getPose().getX(), follower.getPose().getY()));
            telemetry.update();
            follower.update();

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
        public double getthetared(double x1, double y1) {
             return Math.atan2(144-x1, 144-y1);
        }
        public double getthetablue(double x1, double y1) {
            return Math.atan2(0-x1, 144-y1);
        }
}
