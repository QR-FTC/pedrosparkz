package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "farblues", group = "Examples")
public class nearRedLM3V2 extends OpMode {
    boolean gateservoended2 = false;
    boolean case1Started = false;
    boolean case3Started = false;
    boolean case2Started = false;
    boolean case4Started = false;
    boolean gateservoended = false;
    private DcMotor intake;

    private Follower follower;
    private DcMotor shootingmotor;
    private Timer pathTimer, actionTimer, opmodeTimer, catTimer, dogTimer, arrowTimer, bowTimer;
    private int pathState;
    private final Pose startPose = new Pose(120, 128, Math.toRadians(217)); // the robot will be set where the left wheels are along the lines of the beginning of the third tile of x.


    private final Pose scorePose = new Pose(86, 120, Math.toRadians(45)); // left front wheel will be on this point; and its on the 2nd tile in x and fourth tile in y along y=-x.
    //    private final Pose scorePose = new Pose(86, 105, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose arrivingtomiddleballs = new Pose(100, 60, Math.toRadians(0));
    private final Pose collectingmiddleballs = new Pose(120, 60, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose opengate = new Pose(129, 72, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose arrivingtoset1 = new Pose(100, 84, Math.toRadians(0));
    private final Pose collectingset1 = new Pose(120, 84, Math.toRadians(0));

    private final Pose arrivingset3 = new Pose(100, 36, Math.toRadians(0));
    private final Pose collectingset3 = new Pose(120, 36, Math.toRadians(0));
    PathChain scorePreload, gomiddleset, collectmiddleset, openingate, shootmiddleset, arriveset1, collectset1, shootset1, arriveset3, collectset3, scoringset3;


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
                .addPath(new BezierLine(arrivingtomiddleballs, collectingmiddleballs))
                .setLinearHeadingInterpolation(arrivingtomiddleballs.getHeading(), collectingmiddleballs.getHeading())
                .build();
        openingate = follower.pathBuilder()
                .addPath(new BezierLine(collectingmiddleballs, opengate))
                .setLinearHeadingInterpolation(collectingmiddleballs.getHeading(), opengate.getHeading())
                .build();

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
                .setLinearHeadingInterpolation(arrivingtoset1.getHeading(), collectingset1.getHeading())
                .build();
//                /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        shootset1 = follower.pathBuilder()
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
                    if (!case2Started) {
                        case2Started = true;
                        dogTimer.resetTimer();
// "case1Started" is used so that the timers will only start counting once the rest continues and wont reset when it runs over the code again.
                    }
                    if (0.00 <= dogTimer.getElapsedTimeSeconds() && catTimer.getElapsedTimeSeconds() < 0.3) {
                        intake.setPower(-0.8);
                    }
                    if (0.25 <= dogTimer.getElapsedTimeSeconds() && catTimer.getElapsedTimeSeconds() < 0.35) {
                        intake.setPower(0.0);
                        shootingmotor.setPower(-0.7);
                    }
                    if (0.35 <= dogTimer.getElapsedTimeSeconds() && catTimer.getElapsedTimeSeconds() < 2.35) {
                        intake.setPower(0.8);
                    }
                    if (2.35 <= dogTimer.getElapsedTimeSeconds()) {
                        intake.setPower(0.0);
                        shootingmotor.setPower(0.0);
                        setPathState(6);
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
                    intake.setPower(0.8);
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
            case 4: {
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(openingate, true);
                    intake.setPower(0.8);
                    setPathState(5);

                }
                break;
            }
            case 5: {
                if (!follower.isBusy()) {
                    follower.followPath(shootmiddleset, true);
                    setPathState(15);
                }


                break;
            }
            case 15: {
                if (!follower.isBusy()) {
                    if (!case1Started) {
                        catTimer.resetTimer();
                        case1Started = true;
                    }
                    if (0.00 <= catTimer.getElapsedTimeSeconds() && catTimer.getElapsedTimeSeconds() < 0.3) {
                        intake.setPower(-0.8);
                    }
                    if (0.25 <= catTimer.getElapsedTimeSeconds() && catTimer.getElapsedTimeSeconds() < 0.35) {
                        intake.setPower(0.0);
                        shootingmotor.setPower(-0.7);
                    }
                    if (0.35 <= catTimer.getElapsedTimeSeconds() && catTimer.getElapsedTimeSeconds() < 2.35) {
                        intake.setPower(0.8);
                    }
                    if (2.35 <= catTimer.getElapsedTimeSeconds()) {
                        setPathState(6);
                    }
                }
                break;
            }


            case 6: {
                if (!follower.isBusy()) {
                    follower.followPath(arriveset1, true);
                    setPathState(7);
                    intake.setPower(0.8);
                }
                break;
            }
            case 7: {
                if (!follower.isBusy()) {
                    follower.followPath(collectset1, true);
                    setPathState(8);
                }
            }
            break;
            case 8: {
                if (!follower.isBusy()) {
                    follower.followPath(shootset1, true);
                    setPathState(20);
                }
            }
            break;
            case 20: {
                if (!follower.isBusy()) {
                    if (!case3Started) {
                        case3Started = true;
                        arrowTimer.resetTimer();
                    }
                    if (0.00 <= arrowTimer.getElapsedTimeSeconds() && arrowTimer.getElapsedTimeSeconds() < 0.3) {
                        intake.setPower(-0.8);
                    }
                    if (0.25 <= arrowTimer.getElapsedTimeSeconds() && arrowTimer.getElapsedTimeSeconds() < 0.35) {
                        intake.setPower(0.0);
                        shootingmotor.setPower(-0.7);
                    }
                    if (0.35 <= arrowTimer.getElapsedTimeSeconds() && arrowTimer.getElapsedTimeSeconds() < 2.35) {
                        intake.setPower(0.8);
                    }
                    if (2.35 <= arrowTimer.getElapsedTimeSeconds()) {
                        setPathState(9);
                    }
                }
            }
            break;
            case 9: {
                if (!follower.isBusy()) {
                    follower.followPath(arriveset3, true);
                    intake.setPower(0.8);
                    setPathState(10);
                }
            }
            break;

            case 10: {
                if (!follower.isBusy()) {
                    follower.followPath(collectset3, true);
                    setPathState(11);
                }
            }
            break;
            case 11: {
                if (!follower.isBusy()) {
                    follower.followPath(scoringset3, true);
                    setPathState(25);
                }
            }
            break;
            case 25: {
                if (!follower.isBusy()) {
                    if (!case4Started) {
                        bowTimer.resetTimer();
                    }
                    if (0.00 <= bowTimer.getElapsedTimeSeconds() && bowTimer.getElapsedTimeSeconds() < 0.3) {
                        intake.setPower(-0.8);
                    }
                    if (0.25 <= bowTimer.getElapsedTimeSeconds() && bowTimer.getElapsedTimeSeconds() < 0.35) {
                        intake.setPower(0.0);
                        shootingmotor.setPower(-0.7);
                    }
                    if (0.35 <= bowTimer.getElapsedTimeSeconds() && bowTimer.getElapsedTimeSeconds() < 2.35) {
                        intake.setPower(0.8);
                    }
                    if (2.35 <= bowTimer.getElapsedTimeSeconds()) {
                        setPathState(-1);
                    }
                }
            }
        }
    }
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
        shootingmotor = hardwareMap.get(DcMotorEx.class, "deposit");;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        // EH port 1


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
}


