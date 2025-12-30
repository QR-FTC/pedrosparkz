    package org.firstinspires.ftc.teamcode.TeleOp;

    import com.bylazar.telemetry.PanelsTelemetry;
    import com.bylazar.telemetry.TelemetryManager;
    import com.pedropathing.follower.Follower;
    import com.pedropathing.geometry.BezierLine;
    import com.pedropathing.geometry.Pose;
    import com.pedropathing.paths.HeadingInterpolator;
    import com.pedropathing.paths.Path;
    import com.pedropathing.paths.PathChain;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

    import java.util.function.Supplier;

    @TeleOp(name="teleopV2")
    public class teleopV3 extends OpMode {
        private int D;

        private Servo servoIntake;
        private Servo servoDeposit;
        private ElapsedTime SleepTimer = new ElapsedTime();
        private DcMotor geckoWheels;
        private DcMotor intake_2;
        private DcMotor deposit;
        private DcMotor intake_3;
        private static Follower follower;
        public static Pose startingPose; //See ExampleAuto to understand how to use this
        private boolean automatedDrive;
        private Supplier<PathChain> pathChain;
        private TelemetryManager telemetryM;
        private boolean slowMode = false;
        private double slowModeMultiplier = 0.5;

        @Override
        public void init() {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(36, 12, 90));
            follower.update();
            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
            pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                    .build();
            servoDeposit = hardwareMap.get(Servo.class, "servoDeposit"); // CH Port 5
            servoIntake = hardwareMap.get(Servo.class, "servoIntake"); // CH Port 1
            intake_3 = hardwareMap.get(DcMotor.class, "intake_3"); // EH Port 0
            intake_2 = hardwareMap.get(DcMotor.class, "intake_2"); // EH port 1
            deposit = hardwareMap.get(DcMotor.class, "depositMotor"); // EH port 2

        }

        @Override

        public void start() {
            //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
            //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
            //If you don't pass anything in, it uses the default (false)
            follower.startTeleopDrive();
        }

        @Override
        public void loop() {
            SleepTimer.reset();
            //Call this once per loop
            follower.update();

//            double t =2;
//            double d=0.096;
//            double RVelocity = ((2*RDistance)/(39.37*t));
//            double BVelocity = ((2*BDistance)/(39.37*t));
//            double BRPM = (60*BVelocity)/(Math.PI*d);
//            double RRPM = (60*RVelocity) / (Math.PI*d);
//            double r = RRPM/3400;
//            double b = BRPM/3400;
//
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

            // When gamepad-1 right bumper is pressed run intake 1 and 2 motors
            if (gamepad1.right_bumper) {
                intake_2.setPower(0.8);
                intake_3.setPower(-0.8);
            } else if (gamepad1.left_bumper) { // When Gamepad-1 left bumper is pressed reverse the intake motor
                intake_2.setPower(-0.8);
                intake_3.setPower(0.8);

            } else { // if not stop the motor
                intake_2.setPower(0.0);
                intake_3.setPower(0.0);
            }

            // When gamepad-2 right trigger is pressed start deposit motor.
            if (gamepad2.right_trigger > 0.1) {
                deposit.setPower(-0.75 * (gamepad2.right_trigger));
            } else { // else stop the motor
                deposit.setPower(0.0);
            }

            // When Gamepad-2 dpad up is pressed elevator bring the ball up.
            if (gamepad2.dpad_up) {
                servoDeposit.setPosition(0.8);
                while (SleepTimer.milliseconds() < 150) {
                    telemetry.update();
                }
            }
            // When Gamepad-2 dpad down is pressed elevator comes down.
            if (gamepad2.dpad_down) {
                servoDeposit.setPosition(0.3);
                while (SleepTimer.milliseconds() < 150) {
                    telemetry.update();
                }
            }

            if (gamepad2.b) {
                servoIntake.setPosition(0.1);
                while (SleepTimer.milliseconds() < 150) {
                    telemetry.update();
                }
            } else if (gamepad2.a) {
                servoIntake.setPosition(0.2);
                while (SleepTimer.milliseconds() < 150) {
                    telemetry.update();
                }
            }
//            if (gamepad2.x) {
//                servoDeposit.setPosition(0.3);
//                servoIntake.setPosition(0.2);
//                while (SleepTimer.milliseconds() < 150) {
//                    telemetry.update();
//                }
//            }
            //       // if(gamepad2.right_trigger>0.1) {
    //        //    geckoWheels.setPower(gamepad2.right_trigger);
    //        }
    //       //else if (gamepad2.left_trigger>0.1){
    //            geckoWheels.setPower(-gamepad2.left_trigger);
    //        }
    //        else {
    //            geckoWheels.setPower(0.0);
    //
    //        }
    //
    //
    //        if (gamepad2.dpad_up){
    //            servoDeposit.setPower(1);
    //
    //        }
    //        if(gamepad2.dpad_left){
    //            servoDeposit.setPower(0);
    //        }
    //        if(gamepad2.dpad_down){
    //            servoDeposit.setPower(-1);
    //        }
    //        follower.update();
    ////        telemetryM.update();
    //        if (!automatedDrive) {
    //            //Make the last parameter false for field-centric
    //            //In case the drivers want to use a "slowMode" you can scale the vectors
    //            //This is the normal version to use in the TeleOp
    //            if (!slowMode) follower.setTeleOpDrive(
    //                    -gamepad1.left_stick_y,
    //                    -gamepad1.left_stick_x,
    //                    -gamepad1.right_stick_x,
    //                    true // Robot Centric
    //            );
    //            //This is how it looks with slowMode on
    //        }

            telemetry.addData("Deposit Servo Position", servoDeposit.getPosition());
            telemetry.addData("Gate Servo Position", servoIntake.getPosition());
            telemetry.addData("Intake Power Left", intake_2.getPower());
            telemetry.addData("Intake Power Right", intake_3.getPower());
            telemetry.addData("Deposit Power", deposit.getPower());
            telemetry.addData("Current X pos", follower.getPose().getX());
            telemetry.addData("Current Y Pos", follower.getPose().getY());
            telemetry.addData("Current Heading", follower.getPose().getHeading());
            telemetry.addData("Distance",getDistance());
            telemetry.addData("Current RPM", getRPM(getDistance(), 35));
//          telemetry.addData("Distance to Blue Scoring", BDistance);
//          telemetry.addData("Distance to Red Scoring", RDistance);
//          telemetry.addData("Shoot to Red Velocity", RVelocity);
//          telemetry.addData("Shoot to Blue Velocity", BVelocity);
            telemetry.update();

            //
    //        telemetryM.debug("position", follower.getPose());
    //        telemetryM.debug("velocity", follower.getVelocity());
    //        telemetryM.debug("automatedDrive", automatedDrive);

        }

        public double getDistance()
        {
            Pose pose = follower.getPose();
            double targetX = 12;
            double targetY = 132;
            double distanceInches = Math.sqrt(Math.pow((pose.getX() - targetX), 2) + Math.pow(pose.getY() - targetY, 2));
            return distanceInches*25.4;
        }
        public double getRPM(double distance, double theta){

            // Gravity in mm
            double gravityMM = 9806.94;
            double targetHeight = 812.8;
            double thetaRadian = Math.toRadians(theta);

            // v^2 = G*d^2/(2.cos(theta)^2(d.tan(theta)-targetHight)
            double numerator = (gravityMM*distance*distance);
            double denominator = (2* Math.pow(Math.cos(thetaRadian), 2)*(distance*Math.tan(thetaRadian)-targetHeight));

            telemetry.addData("target Height", targetHeight);
            telemetry.addData("Numerator", numerator);
            telemetry.addData("Denominator", denominator);

            double velocity =  numerator / denominator;

            velocity = Math.sqrt(velocity);
            telemetry.addData("velocity", velocity);

            double flywheelDiameter = 96;
            return (60*velocity) / (2*Math.PI*flywheelDiameter);

        }
    }
