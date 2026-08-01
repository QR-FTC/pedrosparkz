package org.firstinspires.ftc.teamcode.Layers.Physical;
import com.qualcomm.robotcore.hardware.CRServo;


public class advancedcrServo {
    private CRServo crServo;
    private boolean reversed;
    int max;
    boolean det = false;

    public advancedcrServo(CRServo servoo) {
        this.crServo= servoo;
    }

    public void setPower(double power) {
        if (reversed) {
            power = -power;

        }
        if (det) {
            if (power >= max) {
                crServo.setPower(max);
            }
            if (power <= -max) {
                crServo.setPower(-max);
            }

            if (power <= max && power >= -max) {
                crServo.setPower(power);
            }
        }
        if (!det) {
            if (power >= 0.8) {
                crServo.setPower(0.8);

            }
            if (power <= -0.8) {
                crServo.setPower(-0.8);
            }
            if (power <= 0.8 && power >= -0.8) {
                crServo.setPower(power);
            }
        }
    }

    public void setReversed(boolean reversed) {
        this.reversed=reversed;

    }

    public void stop() {
        crServo.setPower(0);
    }

    public double getPower() {
        return crServo.getPower();
    }

    public void setMax(int max) {
        this.max=max;
        det = true;
    }
    // limit power



}
