package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.subsystems.Subsystem;

public class intakeSubsystem implements Subsystem {
    DcMotor motor;
    DcMotor motor1;
    double power;
    Telemetry telemetry;


    TouchSensor touchSensor;

    @Override
    public void initialize() {
        motor.setPower(0);
        motor1.setPower(0);

    }



    @Override
    public void periodic() {
        telemetry.addData("Intake Power", motor.getPower());
        telemetry.addData("Touch Sensor", touchSensor.isPressed());
        touchSensor.isPressed();

    }

    public void intake(double power) {
        this.power = power;
        motor.setPower(power);
        motor1.setPower(power);

    }
    public void initialize2(DcMotor motor, DcMotor motor1) {
        this.motor= motor;
        this.motor1= motor1;
    }
    public boolean intakeTouchSensorCheck() {
        return touchSensor.isPressed();
    }


}
