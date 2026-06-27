package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Layers.Physical.dtMotor;

@TeleOp(name = "MecanumDriveEncoder")
public class MecanumDriveEncoderTeleop extends OpMode {
    private dtMotor frontLeft;
    private dtMotor frontRight;
    private dtMotor backLeft;
    private dtMotor backRight;

    // Direct hardware references to the encoder ports (No external libraries required)
    private DcMotorEx xEncoder;
    private DcMotorEx yEncoder;

    @Override
    public void init() {
        frontLeft = new dtMotor(telemetry, hardwareMap, "frontLeft");
        frontRight = new dtMotor(telemetry, hardwareMap,"frontRight");
        backLeft = new dtMotor(telemetry, hardwareMap, "backLeft");
        backRight = new dtMotor(telemetry, hardwareMap, "backRight");

        // Map the deadwheels directly to the ports they are physically wired into
        // Since xEncoder shares a port with frontLeft, map it to the "frontLeft" hardware name
        xEncoder = hardwareMap.get(DcMotorEx.class, "frontLeft");
        yEncoder = hardwareMap.get(DcMotorEx.class, "backLeft");

        // Force the internal hardware counters to start exactly at 0
        xEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Set them to run freely without interference from internal motor velocity formulas
        xEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        yEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y; // Invert depending on your setup
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Calculate raw values
        double frontLeftPower = forward + strafe + turn;
        double backLeftPower = forward - strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backRightPower = forward + strafe - turn;

        // Normalize values so no wheel power exceeds 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

        // Assign powers to motors
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        // Motor Telemetry
        telemetry.addData("Front Left Power", frontLeft.getValue());
        telemetry.addData("Front Right Power", frontRight.getValue());
        telemetry.addData("Back Left Power", backLeft.getValue());
        telemetry.addData("Back Right Power", backRight.getValue());

        // Native hardware methods read live accumulated values automatically
        telemetry.addData("X Encoder (Strafe) Total Pos", xEncoder.getCurrentPosition());
        telemetry.addData("Y Encoder (Forward) Total Pos", yEncoder.getCurrentPosition());

        telemetry.update();
    }
}