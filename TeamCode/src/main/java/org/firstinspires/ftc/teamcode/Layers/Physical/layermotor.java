package org.firstinspires.ftc.teamcode.Layers.Physical;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class layermotor {
    DcMotorEx motor;
    Telemetry telemetry;

    private static final double TICKS_PER_REV = 2786.2;

    public layermotor(Telemetry telemetry, HardwareMap hardwareMap, String name) {

       motor=  hardwareMap.get(DcMotorEx.class, name);

        this.telemetry = telemetry;
    }

    public void setPower(double power) {
        motor.setPower(power);




    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }




    public double getValue(){
        return motor.getPower();
    }


    public void setMode(DcMotor.RunMode mode){
        motor.setMode(mode);
    }
    public void resetEncoder(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }






    public void stopMotor(){
        motor.setPower(0);
    }

}
