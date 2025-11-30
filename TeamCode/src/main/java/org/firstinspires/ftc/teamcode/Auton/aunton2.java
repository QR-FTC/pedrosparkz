package org.firstinspires.ftc.teamcode.Auton;
// this code is for the red side starting at the TOP.
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
@Autonomous(name = "Example Auto closeblue", group = "Examples")
public class aunton2 extends OpMode {
    private Follower follower;
    private DcMotor geckoWheels;
    private Timer pathTimer, actionTimer, opmodeTimer, catTimer, dogTimer;
    private int pathState;
    private CRServo intakeservo;
    private DcMotor intake_2;
    private DcMotor intake_3;
    private final Pose scorePose = new Pose(58, 96, Math.toRadians(45)); //left front wheel will be on the corner of 2nd tile in x direction and fourth in y direction.




//    private final Pose scorePose = new Pose(56, 105, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Posebeg = new Pose(19, 84, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1Pose = new Pose(127, 83, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose getPickup2begPose = new Pose(44, 60, Math.toRadians(0));
    private final Pose getGetPickup2Pose = new Pose(15, 60, Math.toRadians(0));

    private final Pose getPickup3begPose = new Pose(44, 36, Math.toRadians(0));
    private final Pose pickup3pose = new Pose(15, 36, Math.toRadians(0));




    PathChain scorePreload, grabPickup1, scorePickup1a, grabPickup1b, scorePickup2,grabPickup2a,scorePickup2b,scorePickup3, grabPickub3a, grabPickup3b;

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
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, getPickup2begPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),getPickup2begPose .getHeading())
                .build();
//                /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2a = follower.pathBuilder()
                .addPath(new BezierLine(getPickup2begPose, getGetPickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), getGetPickup2Pose.getHeading())
                .build();
        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2b = follower.pathBuilder()
                .addPath(new BezierLine(getGetPickup2Pose, scorePose))
                .setLinearHeadingInterpolation(getGetPickup2Pose.getHeading(), scorePose.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, getPickup3begPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), getPickup3begPose.getHeading())
                .build();
        grabPickub3a = follower.pathBuilder()
                .addPath(new BezierLine(getPickup3begPose, pickup3pose))
                .setLinearHeadingInterpolation(getPickup3begPose.getHeading(), pickup3pose.getHeading())
                .build();
        grabPickup3b = follower.pathBuilder()
                .addPath(new BezierLine(pickup3pose, scorePose))
                .setLinearHeadingInterpolation(pickup3pose.getHeading(), scorePose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: {
                if(!follower.isBusy()) {
                    follower.followPath(scorePreload, true);
                    setPathState(1);
                    geckoWheels.setPower(1);
                    intake_2.setPower(-1);
                    intake_3.setPower(1);
                    intakeservo.setPower(-1);
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
                if (!follower.isBusy() && dogTimer.getElapsedTimeSeconds() > 8.00) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1, true);
                    geckoWheels.setPower(0);
                    intake_2.setPower(0);
                    intake_3.setPower(0);
                    intakeservo.setPower(0);
                    setPathState(2);
                    dogTimer.resetTimer();
                }
            }
            break;
            case 2: {
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1a, true);
                    intake_2.setPower(-1);
                    intake_3.setPower(1);
                    setPathState(3);
                    dogTimer.resetTimer();
                }
            }
            break;
            case 3:
            {
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1b, true);
                    geckoWheels.setPower(1);
                    intake_2.setPower(-1);
                    intake_3.setPower(1);
                    intakeservo.setPower(-1);
                    setPathState(4);
                    dogTimer.resetTimer();

                }
            }
            break;
            case 4: {

                if (!follower.isBusy() && dogTimer.getElapsedTimeSeconds() > 8.00) {
                    follower.followPath(scorePickup2, true);
                    geckoWheels.setPower(0);
                    intake_2.setPower(0);
                    intake_3.setPower(0);
                    intakeservo.setPower(0);
                    setPathState(5);
                    dogTimer.resetTimer();
                }
            }
            break;

            case 5: {
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2a, true);
                    intake_2.setPower(-1);
                    intake_3.setPower(1);
                    setPathState(6);
                    dogTimer.resetTimer();
                }
            }
            break;

            case 6: {
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2b, true);
                    geckoWheels.setPower(1);
                    intake_2.setPower(-1);
                    intake_3.setPower(1);
                    intakeservo.setPower(-1);
                    setPathState(7);
                    dogTimer.resetTimer();
                }
            }
            break;

            case 7: {
                if (!follower.isBusy() && dogTimer.getElapsedTimeSeconds() > 8.00) {

                    follower.followPath(scorePickup3, true);
                    geckoWheels.setPower(0);
                    intake_2.setPower(0);
                    intake_3.setPower(0);
                    intakeservo.setPower(0);
                    setPathState(8);
                    dogTimer.resetTimer();
                }
            }
            break;

            case 8: {
                if (!follower.isBusy()) {
                    follower.followPath(grabPickub3a, true);
                    intake_2.setPower(-1);
                    intake_3.setPower(1);
                    setPathState(9);
                    dogTimer.resetTimer();
                }
            }
            break;

            case 9: {
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3b, true);
                    geckoWheels.setPower(1);
                    intake_2.setPower(-1);
                    intake_3.setPower(1);
                    intakeservo.setPower(-1);
                    setPathState(10);
                }
            }
            break;
//                    case 4:
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
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && dogTimer.getElapsedTimeSeconds() > 5.00) {
                    geckoWheels.setPower(0);
                    intake_2.setPower(0);
                    intake_3.setPower(0);
                    intakeservo.setPower(0);
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                    dogTimer.resetTimer();

                }
                break;
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
        // These loop the movements of the robot, these must be called continuously in order to work
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
        actionTimer = new Timer();
        dogTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        ;
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
        setPathState(0);
    }
    /** We do not use this because everything should automatically disable **/

}




