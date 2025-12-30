//package org.firstinspires.ftc.teamcode.Auton;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//public class nearBlue {@Autonomous(name = "Example Auto", group = "Examples")
//public class farRed extends OpMode {
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    private int pathState;
//    private final Pose startPose = new Pose(20.8, 123.4,Math.toRadians(90)); // Start Pose of our robot.
//    private final Pose scorePose = new Pose(60, 83, Math.toRadians(90)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
//    private final Pose pickup1Pose = new Pose( 12,83, Math.toRadians(90)); // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose pickup2Pose = new Pose(38, 104, Math.toRadians(90)); // Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose pickup3Pose = new Pose(37, 59, Math.toRadians(90)); // Lowest (Third Set) of Artifacts from the Spike Mark.
//    private final Pose pickup4Pose = new Pose(12, 59, Math.toRadians(90)); // Lowest (Third Set) of Artifacts from the Spike Mark.
//    private final Pose pickup5Pose = new Pose(37, 35, Math.toRadians(90));
//    private final Pose pickup6Pose = new Pose(11, 35, Math.toRadians(90));
//
//    private Path scorePreload;
//    private PathChain firstThree, facingRed, firstThree, fTCollected, secondThree, stcollected, lastThree, ltcollected;
//    public void buildPaths() {
//        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//    /* Here is an example for Constant Interpolation
//    scorePreload.setConstantInterpolation(startPose.getHeading()); */
//        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        firstThree = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, scorePose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(),scorePose.getHeading())
//                .build();
//        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        facingRed = follower.pathBuilder()
//                .addPath(new BezierLine(pickup1Pose, scorePose))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
//                .build();
//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        firstThree = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup2Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .build();
//        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//
//        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        fTCollected = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup3Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
//                .build();
//        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        secondThree = follower.pathBuilder()
//                .addPath(new BezierLine(pickup4Pose, scorePose))
//                .setLinearHeadingInterpolation(pickup4Pose.getHeading(), scorePose.getHeading())
//                .build();
//        stcollected = follower.pathBuilder()
//                .addPath(new BezierLine(pickup4Pose, scorePose))
//                .setLinearHeadingInterpolation(pickup4Pose.getHeading(), scorePose.getHeading())
//                .build();
//        lastThree = follower.pathBuilder()
//                .addPath(new BezierLine(pickup5Pose, scorePose))
//                .setLinearHeadingInterpolation(pickup5Pose.getHeading(), scorePose.getHeading())
//                .build();
//        ltcollected = follower.pathBuilder()
//                .addPath(new BezierLine(pickup6Pose, scorePose))
//                .setLinearHeadingInterpolation(pickup6Pose.getHeading(), scorePose.getHeading())
//                .build();
//    }public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                follower.followPath(scorePreload);
//                setPathState(1);
//                break;
//            case 1:
//            /* You could check for
//            - Follower State: "if(!follower.isBusy()) {}"
//            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
//            - Robot Position: "if(follower.getPose().getX() > 36) {}"
//            */
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Preload */
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(middleBlue,true);
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
//                if(!follower.isBusy()) {
//                    /* Grab Sample */
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(facingRed,true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(firstThree,true);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//                if(!follower.isBusy()) {
//                    /* Grab Sample */
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(fTCollected,true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(facingRed,true);
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
//                if(!follower.isBusy()) {
//                    /* Grab Sample */
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(secondThree, true);
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                    follower.followPath(stcollected, true);
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                    follower.followPath(facingRed, true);
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                    follower.followPath(lastThree, true);
//                    setPathState(10);
//                }
//                break;
//            case 10:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                    follower.followPath(ltcollected, true);
//                    setPathState(11);
//                }
//                break;
//            case 11:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                    follower.followPath(facingRed, true);
//                    setPathState(-1);
//                }
//                break;
//        }
//    }
//    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }    @Override
//    public void loop() {
//        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
//        autonomousPathUpdate();
//        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();
//    }
//    /** This method is called once at the init of the OpMode. **/
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);
//    }
//    /** This method is called continuously after Init while waiting for "play". **/
//    @Override
//    public void init_loop() {}
//    /** This method is called once at the start of the OpMode.
//     * It runs all the setup actions, including building paths and starting the path system **/
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//    /** We do not use this because everything should automatically disable **/
//    @Override
//    public void stop() {}
//}
//
//
//
//}
