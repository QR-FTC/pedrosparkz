package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Layers.Physical.dtMotor;

@TeleOp(name = "MecanumDrivePinpoint")
public class MecanumDrivePinpointTeleop extends OpMode {
    private dtMotor frontLeft;
    private dtMotor frontRight;
    private dtMotor backLeft;
    private dtMotor backRight;


    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        frontLeft = new dtMotor(telemetry, hardwareMap, "frontLeft");
        frontRight = new dtMotor(telemetry, hardwareMap,"frontRight");
        backLeft = new dtMotor(telemetry, hardwareMap, "backLeft");
        backRight = new dtMotor(telemetry, hardwareMap, "backRight");


        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        pinpoint.setOffsets(-32.0, 45.0, DistanceUnit.MM);


        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);


        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        telemetry.addLine("Calibrating Pinpoint IMU... DO NOT MOVE ROBOT!");
        telemetry.update();
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void loop() {

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double frontLeftPower = forward + strafe + turn;
        double backLeftPower = forward - strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backRightPower = forward + strafe - turn;


        double max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);


        pinpoint.update();

       
        Pose2D pos = pinpoint.getPosition();

        telemetry.addData("X Position (Inches)", pos.getX(DistanceUnit.INCH));
        telemetry.addData("Y Position (Inches)", pos.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (Degrees)", pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Device Status Indicator", pinpoint.getDeviceStatus());

        telemetry.update();
    }
}