package org.firstinspires.ftc.teamcode.Layers.Physical;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class layermotor {
    static DcMotor motor;
    Telemetry telemetry;

    public void layers(Telemetry telemetry, HardwareMap hardwareMap, String name) {

       motor=  hardwareMap.get(DcMotor.class, name);

        this.telemetry = telemetry;
    }

    public static void setPower(double power) {
        motor.setPower(power);

    }

    public void stopMotor(){
        motor.setPower(0);
    }

}
