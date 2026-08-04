package org.firstinspires.ftc.teamcode.Layers.Physical;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class advancedMotor {
    DcMotor motor;
    public double minPower = -0.8;
    public double maxPower = 0.5;

    public advancedMotor(DcMotor motor) {
        this.motor = motor;
    }

    public void setPower(double power) {
        motor.setPower(power);

    }
    public void setLimit (double min, double max) {
        minPower = min;
        maxPower = max;

    }
    public void setClippedPower(double power) {

        double clippedPower = Range.clip(power, minPower, maxPower);
        motor.setPower(clippedPower);
    }
    public void stopMotor() {
        motor.setPower(0);
    }

    public double getValue() {
        return motor.getPower();

    }

}

