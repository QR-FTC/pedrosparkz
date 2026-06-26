package org.firstinspires.ftc.teamcode.Layers.Physical;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class deadwheel {

    private DcMotorEx encoder;
    private int directionMultiplier = 1;


    private static double TICKS_PER_REV = 8192.0;
    private static  double WHEEL_DIAMETER_INCHES = 1.25984;
    private static final double DISTANCE_PER_TICK = (Math.PI * WHEEL_DIAMETER_INCHES) / TICKS_PER_REV;

    private int offsetTicks = 0;


    public deadwheel(DcMotorEx encoder) {
        this.encoder = encoder;
    }

    public void setDirection(boolean reversed) {
        this.directionMultiplier = reversed ? -1 : 1;
    }

    public int getCurrentPosition() {
        return (encoder.getCurrentPosition() - offsetTicks) * directionMultiplier;
    }

    public double getDistance() {
        return getCurrentPosition() * DISTANCE_PER_TICK;
    }

    public void reset() {
        offsetTicks = encoder.getCurrentPosition();
    }
}