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
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.PIDFCoefficients;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

    import java.util.function.Supplier;

    @TeleOp(name="teleopV3")
    public class teleopV3 extends OpMode {
        public double P = 65;
        public double I =0;
        public double D =0;
        public double F =16.8;
        public double RPM = 0;
        boolean autoShooting = false;
        private ElapsedTime SleepTimer = new ElapsedTime();
        private DcMotorEx deposit;
        private DcMotor intake;
        private static Follower follower;
        private ShooterCalculatons shooterCalculatons;
        public static Pose startingPose; //See ExampleAuto to understand how to use this
        private Supplier<PathChain> pathChain;
        private TelemetryManager telemetryM;
        private double slowModeMultiplier = 0.5;
        private boolean automatedDrive = false;

        @Override
        public void init() {
            shooterCalculatons = new ShooterCalculatons();
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
            follower.update();
            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
            pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, follower::getPose)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, shooterCalculatons.getthetared(follower.getPose().getX(), follower.getPose().getY()), 0.8))
                    .build();
            intake = hardwareMap.get(DcMotor.class, "intake"); // EH Port 0
            deposit = hardwareMap.get(DcMotorEx.class, "deposit"); // EH port 1
            deposit.setDirection(DcMotorSimple.Direction.REVERSE);


            // Tuned vals for P and F
            deposit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            deposit.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
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
            if (!automatedDrive) {
                //Make the last parameter false for field-centric
                follower.setTeleOpDrive(
                        -0.8 * gamepad1.left_stick_y,
                        0.8 * gamepad1.left_stick_x,
                        0.8 * gamepad1.right_stick_x,
                        true // Robot Centric

                );
            }

            // When gamepad-1 right bumper is pressed run intake 1 and 2 motors
            if (gamepad1.right_bumper) {
                intake.setPower(0.8);
            } else if (gamepad1.left_bumper) { // When Gamepad-1 left bumper is pressed reverse the intake motor
                intake.setPower(-0.8);
            } else { // if not stop the motor
                intake.setPower(0.0);
            }
            //Automated PathFollowing
            if (gamepad1.aWasPressed()) {
                follower.followPath(pathChain.get());
                automatedDrive = true;
            }
            //Stop automated following if the follower is done
            if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
                follower.startTeleopDrive();
                automatedDrive = false;
            }
            // When gamepad-2 right trigger is pressed start deposit motor.
            if (gamepad2.right_trigger > 0.1) {
                autoShooting=true;
            }

            if (gamepad2.left_trigger > 0.1) { // else stop the motor
                autoShooting = false;
            }

            RPM = shooterCalculatons.autoshoot(follower.getPose().getX(), follower.getPose().getY(), true);
            double ticksec = shooterCalculatons.rotationsToTicks(RPM);
            if (autoShooting) {
                deposit.setVelocity(ticksec);
                deposit.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
            }
            else {
                deposit.setPower(0);
            }


            //deposit.setVelocity(getRPM(getDistance(), 35));
            //telemetry.addData("Deposit Servo Position", servoDeposit.getPosition());
            //telemetry.addData("Gate Servo Position", servoIntake.getPosition());
            //telemetry.addData("Intake Power Left", intake_2.getPower());
            //telemetry.addData("Intake Power Right", intake_3.getPower());
            telemetry.addData("Deposit Power", deposit.getPower());
            telemetry.addData("Current X pos", follower.getPose().getX());
            telemetry.addData("Current Y Pos", follower.getPose().getY());
            telemetry.addData("Current Heading", follower.getPose().getHeading());
            telemetry.update();

        }

    }
