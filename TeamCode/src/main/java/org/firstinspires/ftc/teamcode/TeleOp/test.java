package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name="test2")


public class test extends OpMode {



    private DcMotor intake;
    private Servo test; //remember to double check config\]
    private TouchSensor touch;
    @Override
    public void init() {


        intake = hardwareMap.get(DcMotor.class, "intake"); //CH Port 0
        test = hardwareMap.get(Servo.class, "test"); //CH Port 0
        touch = hardwareMap.get(TouchSensor.class, "touch"); //CH Port 1
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);






    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {

            intake.setPower(0.5);
        }else if (gamepad1.left_bumper) {
            intake.setPower(-0.5);
        } else {

            intake.setPower(0.0);
            if (gamepad1.right_trigger >0.01) {
                test.setPosition(1.0);

            }
            boolean switchPressed = touch.isPressed();
            if (switchPressed) {
                telemetry.addData("Switch Pressed", switchPressed);


                test.setPosition(0.0);
            } else {

                telemetry.addData("Switch not Pressed", switchPressed);

            }








        }

        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Servo Position" , test.getPosition());
        telemetry.update();



    }
}
