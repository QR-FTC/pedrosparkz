package org.firstinspires.ftc.teamcode.Layers.Physical;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoPhysical {
    private final Servo servo;

    public ServoPhysical(Servo servo) {
        this.servo = servo;
    }
    double aMin =0;
    double aMax =1;
    double bMin = 0;
    double bMax = 300;

    public void setPosition(double position) {
        servo.setPosition(position);
    }
    public double getPosition() {
        return servo.getPosition();
    }
    public void UnitValues(double newaMin, double newaMax, double newbMin, double newbMax){
        aMin = newaMin;
        aMax = newaMax;
        bMin = newbMin;
        bMax = newbMax;
    }
    public void setMappedPosition(double shiftValue){
        double servoposition = bMin + (shiftValue - aMin) * (bMax - bMin) / (aMax - aMin);
        setPosition(servoposition);
    }
    public void servoReverse() {
            servo.setDirection(Servo.Direction.REVERSE);
    }
    public void servoForward(){
        servo.setDirection(Servo.Direction.FORWARD);
    }
}
