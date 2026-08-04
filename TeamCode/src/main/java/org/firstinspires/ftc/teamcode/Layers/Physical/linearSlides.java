package org.firstinspires.ftc.teamcode.Layers.Physical;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Layers.Physical.advancedEncoder;

public class linearSlides {
    DcMotorEx liftMotor;
    advancedEncoder encoder;


    public linearSlides () {
        encoder = new advancedEncoder(liftMotor, 1440, 1, 4);
        encoder.initEncoder();

    }
    public double addDegreesAndPosition () {
        return encoder.getDegrees() + encoder.getPosition();
    }
    public void setPower(double power) {
        liftMotor.setPower(power);
    }



}
