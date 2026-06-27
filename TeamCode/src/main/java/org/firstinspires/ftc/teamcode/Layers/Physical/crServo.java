package org.firstinspires.ftc.teamcode.Layers.Physical;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class crServo {
    CRServo crServo1;
    private boolean reversed;
    public boolean limited;

    public crServo(String name, HardwareMap hardwareMap) {
        crServo1 = hardwareMap.get(CRServo.class, name);
    }

    public void setPower(double power) {
        if (reversed) {
            power=-power;
        }
        if (power>=0.8) {
            crServo1.setPower(0.8);

        }
        if (power<=-0.8) {
            crServo1.setPower(-0.8);
        }
        crServo1.setPower(power);
    }

    public void setReversed(boolean reversed) {
        this.reversed=reversed;

    }

    public void stop() {
        crServo1.setPower(0);
    }

    public double getPower() {
        return crServo1.getPower();
    }
    // limit power



}
