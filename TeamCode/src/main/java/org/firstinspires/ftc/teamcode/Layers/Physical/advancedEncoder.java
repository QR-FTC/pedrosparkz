package org.firstinspires.ftc.teamcode.Layers.Physical;

import com.qualcomm.robotcore.hardware.DcMotor;

public class advancedEncoder {

    DcMotor encoder;
    private double ticksPerRevolution;
    private double gearRatio;
    private double wheelDiameterInches;


    public advancedEncoder (DcMotor  encoder, double ticksPerRevolution, double gearRatio, double wheelDiameterInches) {
        this.encoder = encoder;
        this.ticksPerRevolution = ticksPerRevolution;
        this.gearRatio = gearRatio;
        this.wheelDiameterInches = wheelDiameterInches;

    }


    public void initEncoder () {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public int getPosition () {
        return encoder.getCurrentPosition();
    }
    public double getDegrees() {
        if (ticksPerRevolution == 0) return 0;

        double totalTicksPerOutputRotation = ticksPerRevolution * gearRatio;
        return ((double) getPosition() / totalTicksPerOutputRotation) * 360.0;
    }
   public double getInches () {
        double circumference = Math.PI * wheelDiameterInches;
        return (getDegrees() * circumference) / 360.0;
   }

    }

