package org.firstinspires.ftc.teamcode.Auton;
// this code is for the red side starting at the bottom.
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "farReds", group = "Examples")
public class farRedLM3 extends OpMode {
    boolean gateservoended2 = false;
    boolean case1Started = false;
    boolean case3Started = false;
    boolean case2Started = false;
    boolean case4Started = false;
    boolean gateservoended = false;

    private Follower follower;
    private DcMotor shootingmotor;
    private Timer pathTimer, actionTimer, opmodeTimer, catTimer, dogTimer, arrowTimer, bowTimer;
    private int pathState;
    private DcMotor intake_2;
    private DcMotor intake_3;

    private Servo intakeservo;
    private Servo gateservo;



    private final Pose startPose = new Pose(88, 8, Math.toRadians(90)); // the robot will be set where the left wheels are along the lines of the beginning of the third tile of x.


    private final Pose scorePose = new Pose(86, 120, Math.toRadians(45)); // left front wheel will be on this point; and its on the 2nd tile in x and fourth tile in y along y=-x.
    //    private final Pose scorePose = new Pose(86, 105, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose arrivingtomiddleballs = new Pose (100,60, Math.toRadians(0));
    private final Pose collectingmiddleballs = new Pose(125, 60, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose opengate = new Pose(129, 72, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose arrivingtoset1 = new Pose(100, 84, Math.toRadians(0));
    private final Pose collectingset1 = new Pose(125, 84, Math.toRadians(0));

    private final Pose arrivingset3 = new Pose(100, 36, Math.toRadians(0));
    private final Pose collectingset3 = new Pose(125, 36, Math.toRadians(0));



    PathChain scorePreload, gomiddleset, collectmiddleset,openingate,shootmiddleset, arriveset1,collectset1,shootset1, arriveset3, collectset3, scoringset3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        gomiddleset = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, arrivingtomiddleballs))
                .setLinearHeadingInterpolation(scorePose.getHeading(), arrivingtomiddleballs.getHeading())
                .build();

        collectmiddleset = follower.pathBuilder()
                .addPath(new BezierLine(arrivingtomiddleballs,collectingmiddleballs))
                .setLinearHeadingInterpolation(arrivingtomiddleballs.getHeading(), collectingmiddleballs.getHeading())
                .build();
        openingate = follower.pathBuilder()
                .addPath(new BezierLine(collectingmiddleballs, opengate))
                .setLinearHeadingInterpolation(collectingmiddleballs.getHeading(), opengate.getHeading())
                .build();


        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        shootmiddleset = follower.pathBuilder()
                .addPath(new BezierLine(opengate, scorePose))
                .setLinearHeadingInterpolation(opengate.getHeading(), scorePose.getHeading())
                .build();
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        arriveset1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, arrivingtoset1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), arrivingtoset1.getHeading())
                .build();
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        collectset1 = follower.pathBuilder()
                .addPath(new BezierLine(arrivingtoset1, collectingset1))
                .setLinearHeadingInterpolation(arrivingtoset1.getHeading(),collectingset1 .getHeading())
                .build();
//                /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        shootset1= follower.pathBuilder()
                .addPath(new BezierLine(collectingset1, scorePose))
                .setLinearHeadingInterpolation(collectingset1.getHeading(), scorePose.getHeading())
                .build();
        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        arriveset3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, arrivingset3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), arrivingset3.getHeading())
                .build();
        collectset3 = follower.pathBuilder()
                .addPath(new BezierLine(arrivingset3, collectingset3))
                .setLinearHeadingInterpolation(arrivingset3.getHeading(), collectingset3.getHeading())
                .build();
        scoringset3 = follower.pathBuilder()
                .addPath(new BezierLine(collectingset3, scorePose))
                .setLinearHeadingInterpolation(collectingset3.getHeading(), scorePose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: {
                follower.followPath(scorePreload);
                setPathState(1);
            }
//            case 0: {
//                follower.followPath(scorePreload);
//                setPathState(1);
//                dogTimer.resetTimer();
//                catTimer.resetTimer();
//                boolean case1Started = false;
//            }
//            break;

            case 1: {
                if (!follower.isBusy()) {
                    if (!case1Started) {
                        case1Started = true;
                        catTimer.resetTimer();
                        dogTimer.resetTimer();
// "case1Started" is used so that the timers will only start counting once the rest continues and wont reset when it runs over the code again.
                    }
                    // WILL ADD SHOOTING HERE
                    // used to push the ball further if needed.


//                    if(catTimer.getElapsedTimeSeconds() >= 30.00) {
//                        setPathState(-1);
//                        if(intakeservo.getPosition()==0.3) {
//                            gateservo.setPosition(0.8);
//                        }
//                        else if(gateservo.getPosition() == 0.8) {
//                            if(!gateservoended) {
//                                gateservoended=true;
//                                arrowTimer.resetTimer();
//                            }
//                            intake_2.setPower(-1);
//                            intake_3.setPower(1);
//                            if(arrowTimer.getElapsedTimeSeconds()>0.00 && arrowTimer.getElapsedTime()<1.50) {
//                                intake_3.setPower(0);
//                                intake_2.setPower(0);
//                            }
//                        }
//                    }
                    setPathState(2);
                }
            }
            break;
            case 2: {
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    follower.followPath(gomiddleset, true);
                    setPathState(3);
                    intake_2.setPower(-0.8);
                    intake_3.setPower(0.8);
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup1, true);
//                    intake_2.setPower(0);
//                    intake_3.setPower(0);
//                    shootingmotor.setPower(0);
//                    intakeservo.setPosition(0);
//                    setPathState(3);
//                    dogTimer.resetTimer();
                }
            }
            break;
            case 3: {
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(collectmiddleset, true);
                    setPathState(4);
                }
            }
            break;
            case 4:
            {
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(openingate, true);
                    intake_2.setPower(0);
                    intake_3.setPower(0);
                    setPathState(5);

                }
                break;
            }
            case 5:
            {
                if(!follower.isBusy()) {
                    follower.followPath(shootmiddleset, true);
                    // WILL ADD SHOOTING HERE
                    setPathState(6);
                }
                break;
            }


            case 6:
            {
                if(!follower.isBusy()) {
                    follower.followPath(arriveset1, true);
                    setPathState(7);
                    intake_2.setPower(-0.8);
                    intake_3.setPower(0.8);
                }
                break;
            }
            case 7:
            {
                if(!follower.isBusy()) {
                    follower.followPath(collectset1, true);
                    setPathState(8);
                }
            }
            break;
            case 8:
            {
                if(!follower.isBusy()) {
                    follower.followPath(shootset1, true);
                    intake_2.setPower(0);
                    intake_3.setPower(0);
                    setPathState(9);
                }
            }
            break;
            case 9:
            {
                if(!follower.isBusy()) {
                    follower.followPath(arriveset3, true);
                    intake_3.setPower(0.8);
                    intake_2.setPower(-0.8);
                    setPathState(10);
                }
            }
            break;

            case 10:
            {
                if(!follower.isBusy()) {
                    follower.followPath(collectset3, true);
                    setPathState(11);
                }
            }
            break;
            case 11:
            {
                if(!follower.isBusy()) {
                    follower.followPath(scoringset3, true);
                    setPathState(-1);
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
        arrowTimer =new Timer();
        bowTimer = new Timer();

        dogTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        shootingmotor = hardwareMap.get(DcMotor.class, "depositMotor");
        intakeservo = hardwareMap.get(Servo.class, "servoDeposit");
        intake_2 = hardwareMap.get(DcMotor.class, "intake_2");
        intake_3 = hardwareMap.get(DcMotor.class, "intake_3");
        gateservo = hardwareMap.get(Servo.class, "servoIntake");

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



