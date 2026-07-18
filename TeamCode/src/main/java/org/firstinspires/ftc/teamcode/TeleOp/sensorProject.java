package org.firstinspires.ftc.teamcode.TeleOp;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
@TeleOp(name="sensorProject")

public class sensorProject extends OpMode {
     DcMotor motor;
    CRServo servo;
     TouchSensor sensor;


    @Override
    public void init() {
        motor= hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(CRServo.class, "servo");
        sensor = hardwareMap.get(TouchSensor.class, "sensor");
        servo.setDirection(CRServo.Direction.FORWARD);

    }
    public void start() {

    }

    @Override
    public void loop() {
        if (sensor.isPressed() && gamepad1.right_trigger>=0.1) {
            servo.setPower(0);
            motor.setPower(0);
            telemetry.addData("gate", "at intended position");
        }


        else if(sensor.isPressed()) {
            motor.setPower(0.3);
            servo.setPower(0.4);
            telemetry.addData("touchsensor", "pressed");
        }
        else if (gamepad1.right_trigger>= 0.1) {
            servo.setPower(gamepad1.right_trigger);
            motor.setPower(gamepad1.right_trigger);
        }
        else if (gamepad1.left_trigger>=0.1) {
            servo.setPower(-gamepad1.left_trigger);
            motor.setPower(-gamepad1.right_trigger);
        }

        else {
            motor.setPower(0);
            servo.setPower(0);
            telemetry.addData("touchsensor", "notpressed");
        }
        telemetry.addData("motor", motor.getPower());
        telemetry.addData("servo", servo.getPower());

        telemetry.update();

    }
}
