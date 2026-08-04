package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Layers.Physical.layermotor;

@TeleOp(name="lab2")

public class lab2 extends OpMode {
    layermotor motor;



    @Override
    public void init() {
        motor = new layermotor(telemetry, hardwareMap, "motor1", 99.5);




    }

    @Override
    public void loop() {

        if (gamepad1.a && !motor.isBusy()) {
            motor.run_using_position(5);
        }
        else if(gamepad1.b && !motor.isBusy()){
            motor.run_using_position(2);
        }
        motor.telemetry();
    }
}
