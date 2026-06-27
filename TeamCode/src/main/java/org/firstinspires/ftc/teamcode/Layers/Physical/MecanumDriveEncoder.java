package org.firstinspires.ftc.teamcode.Layers.Physical;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

 // Ensure this is in your teamcode directory


public class MecanumDriveEncoder {
    private dtMotor frontLeft;
    private dtMotor frontRight;
    private dtMotor backLeft;
    private dtMotor backRight;
    private GoBildaPinpointDriver pinpoint;
    private Telemetry telemetry;

    // Constructor to initialize hardware from your OpMode
    public MecanumDriveEncoder(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        frontLeft = new dtMotor(telemetry, hardwareMap, "frontLeft");
        frontRight = new dtMotor(telemetry, hardwareMap, "frontRight");
        backLeft = new dtMotor(telemetry, hardwareMap, "backLeft");
        backRight = new dtMotor(telemetry, hardwareMap, "backRight");

        // Initialize Pinpoint Computer
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Pinpoint Physical Configuration (Adjust offsets in mm to match your robot)
        pinpoint.setOffsets(-50.0, 12.0, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Reset positions to 0
        pinpoint.resetPosAndIMU();
    }

    // Method to handle calculations, motor power execution, and telemetry
    public void drive(double forward, double strafe, double turn) {
        // Must be called at the beginning of every loop cycle to update tracking data
        pinpoint.update();

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

        // Retrieve Odometry Position
        Pose2D pos = pinpoint.getPosition();

        // Subsystem Telemetry
        telemetry.addData("Front Left Power", frontLeft.getValue());
        telemetry.addData("Front Right Power", frontRight.getValue());
        telemetry.addData("Back Left Power", backLeft.getValue());
        telemetry.addData("Back Right Power", backRight.getValue());

        // Pinpoint Odometry Telemetry
        telemetry.addData("Robot X (in)", pos.getX(DistanceUnit.INCH));
        telemetry.addData("Robot Y (in)", pos.getY(DistanceUnit.INCH));
        telemetry.addData("Robot Heading (deg)", pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
    }

    // Optional: Getter if you need to fetch the Pose coordinates outside this class
    public Pose2D getPose() {
        return pinpoint.getPosition();
    }
}