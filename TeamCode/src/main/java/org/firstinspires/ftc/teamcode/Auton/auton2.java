package org.firstinspires.ftc.teamcode.Auton;
// this code is for the blue side starting at the TOP.
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Set;
@Disabled

@Autonomous(name = "Example Auto closered", group = "Examples")
public class auton2 extends OpMode {
    private Follower follower;
    private DcMotor geckoWheels;
    private DcMotor intake_2;
    private DcMotor intake_3;
    private CRServo intakeservo;
    private Timer pathTimer, actionTimer, opmodeTimer, dogTimer, catTimer;
    private int pathState;



    private final Pose scorePose = new Pose(86, 96, Math.toRadians(45)); //right front wheel will be on this point, and it will go along the line of y=x.
//    private final Pose scorePose = new Pose(96, 96, Math.toRadians(135)); //  wheel will be on this point, turned at the 135 deg angle.
    private final Pose pickup1Posebeg = new Pose(44, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1Pose = new Pose(19, 84, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose getPickup2begPose = new Pose(44, 60, Math.toRadians(180));
    private final Pose getGetPickup2Pose = new Pose(15, 60, Math.toRadians(180));

    private final Pose getPickup3begPose = new Pose(44, 36, Math.toRadians(180));
    private final Pose pickup3pose = new Pose(56, 105, Math.toRadians(180));






    PathChain  scorePreload, grabPickup1, scorePickup1a, grabPickup1b, scorePickup2, scorePickup2a, scorePickup2b, scorePickup3, scorePickup3a, scorePickup3b;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */



    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, scorePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), scorePose.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Posebeg))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Posebeg.getHeading())
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1a = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Posebeg, pickup1Pose))
                .setLinearHeadingInterpolation(pickup1Posebeg.getHeading(), pickup1Pose.getHeading())
                .build();
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1b = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath((new BezierLine(scorePose, getPickup2begPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), getPickup2begPose.getHeading())
                .build();
        scorePickup2a = follower.pathBuilder()
                .addPath((new BezierLine(getPickup2begPose, getGetPickup2Pose)))
                .setLinearHeadingInterpolation(getPickup2begPose.getHeading(), getGetPickup2Pose.getHeading())
                .build();
        scorePickup2b = follower.pathBuilder()
                .addPath((new BezierLine(getGetPickup2Pose,scorePose)))
                .setLinearHeadingInterpolation(getGetPickup2Pose.getHeading(), scorePose.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath((new BezierLine(scorePose,getPickup3begPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), getPickup3begPose.getHeading())
                .build();
        scorePickup3a = follower.pathBuilder()
                .addPath((new BezierLine(getPickup3begPose,pickup3pose)))
                .setLinearHeadingInterpolation(getPickup3begPose.getHeading(), pickup3pose.getHeading())
                .build();
        scorePickup3b = follower.pathBuilder()
                .addPath((new BezierLine(pickup3pose, scorePose)))
                .setLinearHeadingInterpolation(pickup3pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//                scorePickup2 = follower.pathBuilder()
//                        .addPath(new BezierLine(scorePose, getPickup2begPose))
//                        .setLinearHeadingInterpolation(scorePose.getHeading(),getPickup2begPose .getHeading())
//                        .build();
//                /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//                grabPickup3 = follower.pathBuilder()
//                        .addPath(new BezierLine(getPickup2begPose, getGetPickup2Pose))
//                        .setLinearHeadingInterpolation(scorePose.getHeading(), getGetPickup2Pose.getHeading())
//                        .build();
//                /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//                scorePickup3 = follower.pathBuilder()
//                        .addPath(new BezierLine(getGetPickup2Pose, scorePose))
//                        .setLinearHeadingInterpolation(getGetPickup2Pose.getHeading(), scorePose.getHeading())
//                        .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {



            case 0: {
                if (!follower.isBusy() && dogTimer.getElapsedTimeSeconds() > 5.00) {
                    follower.followPath(grabPickup1);
                    intakeservo.setPower(0);
                    intake_2.setPower(0);
                    intake_3.setPower(0);
                    geckoWheels.setPower(0);
                    setPathState(1);
                    dogTimer.resetTimer();
                }
            }
            break;
            case 1: {
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    dogTimer.resetTimer();
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup1a, true);
                    intakeservo.setPower(0);
                    intake_2.setPower(-1);
                    intake_3.setPower(1);
                    geckoWheels.setPower(0);
                    setPathState(2);
                }
            }
            break;

            case 2: {
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                    follower.followPath(grabPickup1b, true);
                    intakeservo.setPower(-1);
                    geckoWheels.setPower(1);
                    intake_3.setPower(1);
                    intake_2.setPower(-1);
                    setPathState(3);

                }
            }
            break;

            case 3: {
                if (!follower.isBusy() && dogTimer.getElapsedTimeSeconds() > 5.00) {
                    follower.followPath(scorePickup2);
                    setPathState(4);
                    geckoWheels.setPower(0);
                    intake_3.setPower(0);
                    intake_2.setPower(0);
                    intakeservo.setPower(0);
                    dogTimer.resetTimer();
                }
            }
            break;
            case 4: {
                if (!follower.isBusy()) {
                    follower.followPath((scorePickup2a));
                    intake_3.setPower(1);
                    intake_2.setPower(-1);
                    setPathState(5);
                    dogTimer.resetTimer();
                }
            }
            break;

            case 5: {
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2b);
                    setPathState(6);
                    geckoWheels.setPower(1);
                    intake_3.setPower(1);
                    intake_2.setPower(-1);
                    intakeservo.setPower(-1);
                }
            }
            break;

            case 6: {
                if (!follower.isBusy() && dogTimer.getElapsedTimeSeconds() > 5.00) {
                    follower.followPath(scorePickup3);
                    setPathState(7);
                    geckoWheels.setPower(0);
                    intake_3.setPower(0);
                    intake_3.setPower(0);
                    intakeservo.setPower(0);
                    dogTimer.resetTimer();
                }
            }
            break;

            case 7: {
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3a);
                    intake_2.setPower(-1);
                    intake_3.setPower(1);
                    setPathState(8);
                    dogTimer.resetTimer();
                }
            }
            break;

            case 8: {
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3b);
                    intake_2.setPower(-1);
                    intake_3.setPower(1);
                    intakeservo.setPower(-1);
                    setPathState(9);
                    geckoWheels.setPower(1);
                    dogTimer.resetTimer();
                }
            }
            break;

            case 9: {
                if (!follower.isBusy() && dogTimer.getElapsedTimeSeconds() > 5.00) {
                    setPathState(-1);
                    geckoWheels.setPower(0);
                    intakeservo.setPower(0);
                    intake_2.setPower(0);
                    intake_3.setPower(0);
                    dogTimer.resetTimer();

                }
            }
            break;

//                        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//                        if(!follower.isBusy()) {
//                            /* Grab Sample */
//                            /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                            follower.followPath(scorePickup2,true);
//                            setPathState(5);
//                        }
//                        break;
//                    case 5:
//                        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                        if(!follower.isBusy()) {
//                            /* Score Sample */
//                            /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                            follower.followPath(grabPickup3,true);
//                            setPathState(6);
//                        }
//                        break;
//                    case 6:
//                        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
//                        if(!follower.isBusy()) {
//                            /* Grab Sample */
//                            /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                            follower.followPath(scorePickup3, true);
//                            setPathState(7);
//                        }
//                        break;

        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        boolean initialRunDone = false;
        if (!initialRunDone) {
            if (opmodeTimer.getElapsedTimeSeconds() < 8.0) {
                intakeservo.setPower(-1);
                intake_2.setPower(-1);
                intake_3.setPower(1);
                geckoWheels.setPower(1);
            } else {
                // Stop motors after 8 seconds and mark as done
                intakeservo.setPower(0);
                intake_2.setPower(0);
                intake_3.setPower(0);
                geckoWheels.setPower(0);
                initialRunDone = true; // only happens once
            }
        } else {
            // After initial run, update follower and paths
            follower.update();
            autonomousPathUpdate();
        }
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        catTimer = new Timer();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        dogTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(scorePose);
        geckoWheels = hardwareMap.get(DcMotor.class, "Deposit");
        intake_2 = hardwareMap.get(DcMotor.class, "Intake_2");
        intake_3 = hardwareMap.get(DcMotor.class, "intake_3");
        intakeservo = hardwareMap.get(CRServo.class, "Servo_Deposit");


    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        dogTimer.resetTimer();
        intakeservo.setPower(-1);
        intake_2.setPower(-1);
        intake_3.setPower(1);
        geckoWheels.setPower(1);
        if (dogTimer.getElapsedTimeSeconds() > 8.00) {
            intakeservo.setPower(0);
            intake_2.setPower(0);
            intake_3.setPower(0);
            geckoWheels.setPower(0);
            setPathState(0);
        }

        setPathState(14);
//        setPathState(0);
//        setPathState(1);
//        setPathState(2);
//        dogTimer.resetTimer();
//        shootingmotor.setPower(1);
//        if (dogTimer.getElapsedTimeSeconds() > 6.00) {
//            shootingmotor.setPower(0);
//        }
//
//            setPathState(3);
//            setPathState(4);
//                setPathState(5);
//            dogTimer.resetTimer();
//            shootingmotor.setPower(1);
//            if (dogTimer.getElapsedTimeSeconds() > 4.00) {
//                shootingmotor.setPower(0);
//            }
//                setPathState(6);
//                setPathState(7);
//                setPathState(8);
//                shootingmotor.setPower(1);
//                if (dogTimer.getElapsedTimeSeconds() > 4.00) {
//                    shootingmotor.setPower(0);
//                }
//                setPathState(9);





    }



    /** We do not use this because everything should automatically disable **/

}



