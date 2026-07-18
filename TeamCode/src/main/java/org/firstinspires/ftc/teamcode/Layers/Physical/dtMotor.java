package org.firstinspires.ftc.teamcode.Layers.Physical;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class dtMotor {
    DcMotor motor;
    Telemetry telemetry;

    public dtMotor(Telemetry telemetry, HardwareMap hardwareMap, String name){
        motor = hardwareMap.get(DcMotor.class, name);

        this.telemetry = telemetry;
    }

    public void setPower(double power){
        motor.setPower(power);
    }

    public void stopMotor(){
        motor.setPower(0);
    }

    public double getValue(){
        return motor.getPower();
    }
}
