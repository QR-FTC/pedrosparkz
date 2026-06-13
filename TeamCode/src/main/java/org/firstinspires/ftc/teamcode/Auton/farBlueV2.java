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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TeleOp.ShooterCalculatons;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "farblueV2", group = "Examples")
public class farBlueV2 extends OpMode {
    boolean gateservoended2 = false;
    boolean case1Started = false;
    boolean case3Started = false;
    boolean case2Started = false;
    boolean case4Started = false;
    boolean case5Started = false;
    private ShooterCalculatons shooterCalculatons;
    boolean gateservoended = false;
    private DcMotor intake;

    private Follower follower;
    private DcMotorEx shootingmotor;
    public double RPM = 0;
    private Timer pathTimer, actionTimer, opmodeTimer, catTimer, dogTimer, arrowTimer, bowTimer, intaketimer;
    private int pathState;


    private final Pose endingposition = new Pose(48, 25, Math.toRadians(90));
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // the robot will be set where the left wheels are along the lines of the beginning of the third tile of x.


    private final Pose scorePose = new Pose(56, 16, Math.toRadians(215));
    private final Pose scorePose1 = new Pose(56,16, Math.toRadians(210));
    private final Pose scorePose2 = new Pose(56,16, Math.toRadians(215));// left front wheel will be on this point; and its on the 2nd tile in x and fourth tile in y along y=-x.
    //    private final Pose scorePose = new Pose(86, 105, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose arrivingtomiddleballs = new Pose (56,60, Math.toRadians(180));
    private final Pose collectingmiddleballs = new Pose(24, 60, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose opengate = new Pose(15, 75, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose arrivingtoset1 = new Pose(46, 84, Math.toRadians(180));
    private final Pose collectingset1 = new Pose(24, 84, Math.toRadians(180));

    private final Pose arrivingset3 = new Pose(56, 36, Math.toRadians(180));
    private final Pose collectingset3 = new Pose(24, 36, Math.toRadians(180));



    PathChain scorePreload, endingpos, gomiddleset, collectmiddleset,openingate,shootmiddleset, arriveset1,collectset1,shootset1, arriveset3, collectset3, scoringset3;

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
                .addPath(new BezierLine(opengate, scorePose1))
                .setLinearHeadingInterpolation(opengate.getHeading(), scorePose1.getHeading())
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
                .addPath(new BezierLine(collectingset1, scorePose2))
                .setLinearHeadingInterpolation(collectingset1.getHeading(), scorePose2.getHeading())
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
        endingpos = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endingposition))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endingposition.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0: {
                follower.followPath(scorePreload);
                RPM=shooterCalculatons.autoshoot(follower.getPose().getX(),follower.getPose().getY(),false)+130;
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
                    if (1.75 <= dogTimer.getElapsedTimeSeconds() && catTimer.getElapsedTimeSeconds() < 4.00) {
                        intake.setPower(-0.8);
                    }
                    if (4.00 <= dogTimer.getElapsedTimeSeconds()) {
                        intake.setPower(0.0);
                        RPM = -1000;
                        setPathState(2);
                    }
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
                    if(!case5Started) {
                        intaketimer.resetTimer();
                        case5Started = true;
                    }
                    follower.followPath(arriveset3, true);
                    setPathState(3);
                    RPM = -1500;
                    intake.setPower(-1);
                    pathTimer.resetTimer();

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
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2.00) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(collectset3, true);
                    setPathState(5);
                    pathTimer.resetTimer();
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
                    intake.setPower(-0.8);
                    setPathState(782);

                }
                break;
            }
            case 5:
            {
                if(!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>2.00) {
                    follower.followPath(scoringset3, true);
                    setPathState(15);
                }


                break;
            }
            case 15:
            {
                if(!follower.isBusy()) {
                    if (!case1Started) {
                        catTimer.resetTimer();
                        case1Started = true;
                    }
                    if (0.00 <= catTimer.getElapsedTimeSeconds() && catTimer.getElapsedTimeSeconds() < 0.3) {
                        intake.setPower(0.3);
                        RPM= -1500;
                    }
                    if (1.0 <= catTimer.getElapsedTimeSeconds() && catTimer.getElapsedTimeSeconds() < 4.25){
                        intake.setPower(0.0);
                        RPM=shooterCalculatons.autoshoot(follower.getPose().getX(),follower.getPose().getY(),false) +130;
                    }
                    if (4.25 <= catTimer.getElapsedTimeSeconds()&& catTimer.getElapsedTimeSeconds()<6.25) {
                        intake.setPower(-0.8);
                    }
                    if(catTimer.getElapsedTimeSeconds()>=6.25) {
                        RPM = -600;
                        intake.setPower(0.0);
                        setPathState(6);

                    }
                }
                break;
            }




            case 6:
            {
                if(!follower.isBusy() ) {
                    follower.followPath(gomiddleset, true);
                    setPathState(7);
                   intake.setPower(-1);
                }
                break;
            }
            case 7:
            {
                if(!follower.isBusy()) {
                    follower.followPath(collectmiddleset, true);
                    setPathState(8);
                    pathTimer.resetTimer();
                }
            }
            break;
            case 8:
            {
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2.00) {
                    follower.followPath(shootmiddleset, true);
                    setPathState(20);
                }
            }
            break;
            case 20:
            {
                if(!follower.isBusy()) {
                    if(!case3Started) {
                        case3Started = true;
                        arrowTimer.resetTimer();
                    }
                    if (0.00 <= arrowTimer.getElapsedTimeSeconds() && arrowTimer.getElapsedTimeSeconds() < 0.3) {
                       intake.setPower(0.3);
                        RPM= -1500;
                    }
                    if (1.0 <= arrowTimer.getElapsedTimeSeconds() && arrowTimer.getElapsedTimeSeconds() < 4.25){
                        intake.setPower(0.0);
                        RPM=shooterCalculatons.autoshoot(follower.getPose().getX(),follower.getPose().getY(),false) +130;
                    }
                    if (4.75 <= arrowTimer.getElapsedTimeSeconds()&& arrowTimer.getElapsedTimeSeconds()<7) {
                        intake.setPower(-0.8);
                    }
                    if(arrowTimer.getElapsedTimeSeconds()>=7) {
                        RPM = -600;
                        intake.setPower(0.0);
                        setPathState(9);

                    }
                }
            }
            break;
            case 9:
            {
                if(!follower.isBusy()) {
                    follower.followPath(endingpos, true);
                   intake.setPower(0.0);
                   RPM = 0;
                    setPathState(-500);
                }
            }
            break;

            case 10:
            {
                if(!follower.isBusy()) {
                    follower.followPath(collectset1, true);
                    intake.setPower(0.0);
                    setPathState(11);
                }
            }
            break;
            case 11:
            {
                if(!follower.isBusy()) {
                    follower.followPath(shootset1, true);
                    intake.setPower(0);
                    setPathState(25);
                }
            }
            break;
            case 25:
            {
                if(!follower.isBusy()) {
                    if(!case4Started) {
                        bowTimer.resetTimer();
                    }
                    if (0.00 <= bowTimer.getElapsedTimeSeconds() && bowTimer.getElapsedTimeSeconds() < 0.3) {
                        intake.setPower(-0.8);
                    }
                    if (0.25 <= bowTimer.getElapsedTimeSeconds() && bowTimer.getElapsedTimeSeconds() < 0.35) {
                        intake.setPower(0.8);
                        RPM=shooterCalculatons.autoshoot(follower.getPose().getX(),follower.getPose().getY(),false);
                    }
                    if (0.35 <= bowTimer.getElapsedTimeSeconds() && bowTimer.getElapsedTimeSeconds() < 2.35) {
                        intake.setPower(0.8);
                    }
                    if (2.35 <= bowTimer.getElapsedTimeSeconds()) {
                        RPM = -1;
                        intake.setPower(0);
                        setPathState(-1);
                    }
//                    if (0.00 <= bowTimer.getElapsedTimeSeconds() && bowTimer.getElapsedTimeSeconds() < 0.3) {
//                        intake.setPower(-0.8);
//                    }
//                    if (0.25 <= bowTimer.getElapsedTimeSeconds() && bowTimer.getElapsedTimeSeconds() < 0.35) {
//                        intake.setPower(0.0);
//                        shootingmotor.setPower(-0.7);
//                    }
//                    if (0.35 <= bowTimer.getElapsedTimeSeconds() && bowTimer.getElapsedTimeSeconds() < 2.35) {
//                        intake.setPower(0.8);
//                    }
//                    if (2.35 <= bowTimer.getElapsedTimeSeconds()) {
//                        setPathState(-1);
//                    }
                }
            }

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
        double ticks = shooterCalculatons.rotationsToTicks(RPM);
        shootingmotor.setVelocity(ticks);
        final double P = 65;
        final double F = 16.8;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shootingmotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
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
        shooterCalculatons = new ShooterCalculatons();
        catTimer = new Timer();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        arrowTimer =new Timer();
        bowTimer = new Timer();
        intaketimer = new Timer();

        dogTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.8);
        shootingmotor = hardwareMap.get(DcMotorEx.class, "deposit");;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        final double P = 65;
        final double F = 16.8;
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shootingmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shootingmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shootingmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shootingmotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
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
    /** We do not use this because everything should automatically disable **/

}



