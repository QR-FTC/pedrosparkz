package org.firstinspires.ftc.teamcode.TeleOp;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name="servo")
public class servo extends OpMode {
    private Servo Gateservo;


    @Override
    public void init() {
        Gateservo = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) {
            Gateservo.setPosition(Gateservo.getPosition()+0.05);
        }
        if (gamepad1.dpadDownWasPressed()) {
            Gateservo.setPosition(Gateservo.getPosition()-0.05);
        }
        telemetry.addData("Servo position", Gateservo.getPosition());
        telemetry.update();

    }
}
