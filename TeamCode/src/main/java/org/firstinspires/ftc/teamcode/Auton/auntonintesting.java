package org.firstinspires.ftc.teamcode.Auton;
// this code is for the red side starting at the bottom.
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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Set;
@Disabled
@Autonomous(name = "Example Auto farblue", group = "Examples")
public class auntonintesting extends OpMode {
    boolean case1Started = false;

    private Follower follower;
    private DcMotor shootingmotor;
    private Timer pathTimer, actionTimer, opmodeTimer, catTimer, dogTimer;
    private int pathState;
    private DcMotor intake_2;
    private DcMotor intake_3;

    public Servo intakeservo;


    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // the robot will be set where the left wheels are along the lines of the beginning of the third tile of x.


    private final Pose scorePose = new Pose(58, 96, Math.toRadians(135)); // left front wheel will be on this point; and its on the 2nd tile in x and fourth tile in y along y=-x.
    //    private final Pose scorePose = new Pose(86, 105, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Posebeg = new Pose(100, 83, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1Pose = new Pose(127, 83, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose getPickup2begPose = new Pose(103, 60, Math.toRadians(0));
    private final Pose getGetPickup2Pose = new Pose(127, 60, Math.toRadians(0));

    private final Pose getPickup3begPose = new Pose(103, 35, Math.toRadians(0));
    private final Pose pickup3pose = new Pose(127, 35, Math.toRadians(0));


    private Path scorePreload;

    PathChain grabPickup1, scorePickup1a, grabPickup1b, scorePickup2,grabPickup2a,scorePickup2b,scorePickup3, grabPickub3a, grabPickup3b;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */



        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
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
                follower.followPath(scorePreload);
                setPathState(1);
                dogTimer.resetTimer();
                catTimer.resetTimer();
                boolean case1Started = false;
            }
            break;

            case 1: {
                if (!follower.isBusy()) {
                    if (!case1Started) {
                        case1Started=true;
                        catTimer.resetTimer();
                        dogTimer.resetTimer();
// "case1Started" is used so that the timers will only start counting once the rest continues and wont reset when it runs over the code again.
                    }

                    intake_2.setPower(-1);
                    intake_3.setPower(1);
                    // used to push the ball further if needed.
                    shootingmotor.setPower(1);
                    if ( 3.50 <= catTimer.getElapsedTimeSeconds() && catTimer.getElapsedTimeSeconds() < 6.50) {
                        intakeservo.setPosition(0.8);
                        // it takes around 3.50 
                    }
                    if (catTimer.getElapsedTimeSeconds() >= 6.50 && catTimer.getElapsedTimeSeconds()<9.50) {
                        intakeservo.setPosition(0);
                    }
                    if (catTimer.getElapsedTimeSeconds() >= 9.50 && catTimer.getElapsedTimeSeconds() <12.50) {
                        intakeservo.setPosition(0.8);
                    }
                    if(catTimer.getElapsedTimeSeconds() >= 12.50 && catTimer.getElapsedTimeSeconds() < 15.50)
                        intakeservo.setPosition(0);
                    if(catTimer.getElapsedTimeSeconds() >= 15.50 && catTimer.getElapsedTimeSeconds() < 16.00) {
                        intakeservo.setPosition(0.8);
                    }


                    if(catTimer.getElapsedTimeSeconds() >= 16.00) {
                        setPathState(2);
                    }
                }

                break;
            }
            case 2: {
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy() && dogTimer.getElapsedTimeSeconds() >= 16.50) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1, true);
                    intake_2.setPower(0);
                    intake_3.setPower(0);
                    shootingmotor.setPower(0);
                    intakeservo.setPosition(0);
                    setPathState(3);
                    dogTimer.resetTimer();
                }
            }
            break;
            case 3: {
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1a, true);
                    setPathState(5);
                    intake_2.setPower(-0.8);
                    intake_3.setPower(0.8);
                    dogTimer.resetTimer();
                }
            }
            break;
            case 5:
            {
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1b, true);
                    intake_2.setPower(0);
                    intake_3.setPower(0);
                    setPathState(6);

                }
            }
            break;
//            case 4: {
//
//                if (!follower.isBusy() && dogTimer.getElapsedTimeSeconds() > 5.00) {
//                    follower.followPath(scorePickup2, true);
//                    setPathState(5);
//                    shootingmotor.setPower(0);
//                    dogTimer.resetTimer();
//                }
//            }
//                break;
//
//            case 5: {
//                if (!follower.isBusy()) {
//                    follower.followPath(grabPickup2a, true);
//                    setPathState(6);
//                    dogTimer.resetTimer();
//                }
//            }
//                break;
//
//            case 6: {
//                if (!follower.isBusy()) {
//                    follower.followPath(scorePickup2b, true);
//                    shootingmotor.setPower(1);
//                    setPathState(7);
//                }
//            }
//            break;
//
//            case 7: {
//                if (!follower.isBusy() && dogTimer.getElapsedTimeSeconds() > 5.00) {
//
//                    follower.followPath(scorePickup3, true);
//                    shootingmotor.setPower(0);
//                    setPathState(8);
//                    dogTimer.resetTimer();
//                }
//            }
//                break;
//
//            case 8: {
//                if (!follower.isBusy()) {
//                    follower.followPath(grabPickub3a, true);
//                    setPathState(9);
//                    dogTimer.resetTimer();
//                }
//            }
//                break;
//
//            case 9: {
//                if (!follower.isBusy()) {
//                    follower.followPath(grabPickup3b, true);
//                    shootingmotor.setPower(1);
//                    setPathState(10);
//                }
//            }
            // break;
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
                    shootingmotor.setPower(0);
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
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
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
        buildPaths();
        follower.setStartingPose(startPose);
        shootingmotor = hardwareMap.get(DcMotor.class, "Deposit");
        intakeservo = hardwareMap.get(Servo.class, "Servo_Deposit");
        intake_2 = hardwareMap.get(DcMotor.class, "Intake_2");
        intake_3 = hardwareMap.get(DcMotor.class, "intake_3");

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



