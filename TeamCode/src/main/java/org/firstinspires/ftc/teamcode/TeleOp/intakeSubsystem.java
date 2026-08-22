package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.subsystems.Subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;



public class intakeSubsystem implements Subsystem {
    private DcMotor motor2;
    private DcMotor motor1;
    double power;
    Telemetry telemetry;
    TouchSensor touchSensor;

    @Override
    public void initialize() {
        motor2.setPower(0);
        motor1.setPower(0);

    }

    @Override
    public void periodic() {
        telemetry.addData("Intake Power", motor2.getPower());
        telemetry.addData("Touch Sensor", touchSensor.isPressed());
        telemetry.update();
        touchSensor.isPressed();

    }

    public void intake(double power) {
        this.power = power;
        motor2.setPower(power);
        motor1.setPower(power);

    }
    public void intakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry, DcMotor motor, DcMotor motor3) {
        this.motor2 = motor;
        motor2 = hardwareMap.get(DcMotor.class, "motor");
        this.motor1 = motor3;
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
    }

    public void intake1run(double power) {
        this.power = power;
        motor1.setPower(power);
        motor2.setPower(0);
    }

    public void stop() {
        motor1.setPower(0);
        motor2.setPower(0);
    }
    public boolean intakeTouchSensorCheck() {
        return touchSensor.isPressed();
    }


}
