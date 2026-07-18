package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Layers.Physical.layermotor;

import com.pedropathing.util.Timer;


@TeleOp(name="layerTest")

public class layerTest extends OpMode {
    private DcMotor motor;
    private Timer timer;

    @Override
    public void init() {
layermotor motor = new layermotor();
motor.layers(telemetry, hardwareMap, "motor1");
motor.setDirection(DcMotorSimple.Direction.FORWARD);
timer= new Timer();
    }

    @Override
    public void loop() {
        if(timer.getElapsedTimeSeconds() >= 5.00) {
            mo
        }
        layermotor.setPower(0.5);

        telemetry.update();

    }
}
