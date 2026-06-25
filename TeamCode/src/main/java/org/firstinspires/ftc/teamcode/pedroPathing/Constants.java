package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-40.570119614401065)
            .lateralZeroPowerAcceleration(-57.34824265816559)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.13, 0 , 0, 0.007))
            .headingPIDFCoefficients(new PIDFCoefficients(0.4, 0, 0.01,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.0003, 0.01, 0.6))
//            .forwardZeroPowerAcceleration(-28.080875034763633)
//            .lateralZeroPowerAcceleration(-60.7282772090663)
//            .headingPIDFCoefficients(new PIDFCoefficients(0.4, 0, 0.000095, 0.1))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.15, 0, 0.001, 0.6, 0.03))
//            .centripetalScaling(0.7)
//            .forwardZeroPowerAcceleration(-49.66762831254822)
//            .lateralZeroPowerAcceleration(-72.07809969149518)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0, 0, 0))
//            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.06, 0.01))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.0008, 0.01, 0.6))
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.0095 , 0))
            .mass(10);


//            .forwardZeroPowerAcceleration()
//            .lateralZeroPowerAcceleration(-53.84367728103716)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.006, 0))
//            .headingPIDFCoefficients(new PIDFCoefficients(0.00006, 0.01, 0, 0.49))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.017, 0, 0.00000001, 0.06, 0.01))
//            .centripetalScaling(0.02)
//            .mass(7);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .strafePodX(-4.5)
            .forwardPodY(-1.75)
//            .forwardPodY(2)
//            .strafePodX(0)

//            .forwardPodY(-1.3)
//            .strafePodX(-4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(85.70949764702263)
            .yVelocity( 71.81979394146778)
//            .xVelocity(81.59892308993602)
//            .yVelocity(66.4411025159941)
//            .xVelocity(91.38599305640994)
//            .yVelocity(74.41339928334153)
//            .xVelocity(91.63383231576033)
//            .yVelocity( 76.13528947004183)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}
