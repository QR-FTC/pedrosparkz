package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Layers.Physical.dtMotor;

@TeleOp(name="motorTest")
public class MotorTest extends OpMode {

    dtMotor motor;

    @Override
    public void init() {
        motor = new dtMotor(telemetry, hardwareMap,"motor");
    }

    @Override
    public void loop() {
        if(gamepad1.right_trigger > 0.01){
            motor.setPower(gamepad1.right_trigger);
        } else if(gamepad1.left_trigger > 0.01){
            motor.setPower(-gamepad1.left_trigger);
        } else {
            motor.stopMotor();
        }
    }
}
