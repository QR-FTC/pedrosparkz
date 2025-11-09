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
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Set;

@Autonomous(name = "Example Auto redside:)", group = "Examples")
public class autonmoving extends OpMode {

    private Follower follower;
    private DcMotor shootingmotor;
    private Timer pathTimer, actionTimer, opmodeTimer, catTimer, dogTimer;
    private int pathState;
    private DcMotor intake_2;
    private DcMotor intake_3;

    public CRServo intakeservo;


    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // the robot will be set where the left wheels are along the lines of the beginning of the third tile of x.



    //    private final Pose scorePose = new Pose(86, 105, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Posebeg = new Pose(58, 12, Math.toRadians(90)); // Highest (First Set) of Artifacts from the Spike Mark.


    private Path scorePreload;

    PathChain grabPickup1, scorePickup1a, grabPickup1b, scorePickup2,grabPickup2a,scorePickup2b,scorePickup3, grabPickub3a, grabPickup3b;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */


        scorePreload = new Path(new BezierLine(startPose, pickup1Posebeg));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), pickup1Posebeg.getHeading());
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: {
                follower.followPath(scorePreload);
                setPathState(-1);

                break;
            }
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
        follower.setStartingPose(startPose);
        buildPaths();
        shootingmotor = hardwareMap.get(DcMotor.class, "Deposit");
        intakeservo = hardwareMap.get(CRServo.class, "Servo_Deposit");
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




