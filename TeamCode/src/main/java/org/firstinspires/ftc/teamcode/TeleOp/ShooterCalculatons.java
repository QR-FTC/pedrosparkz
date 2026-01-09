package org.firstinspires.ftc.teamcode.TeleOp;

public class ShooterCalculatons {
public double ticksToRotations(double ticksPerSecond){
    // EQUATION: tics/second = rotations/minutes. Multiply seconds times 60(conversion)to get minutes
    // then divide Ticks by 28(conversion) to get rotations. TPR = 28
    double rotationsPerMinute = ticksPerSecond / 28 * 60;
    return rotationsPerMinute;

}
public double rotationsToTicks(double rotationsPerMinute){
    // Equation: rotaions/minutes = ticks/seconds. THIS TIME, multiply rotations times 28 to get ticks.
    // then divide by 60.
    double ticksPerSecond = rotationsPerMinute*28 / 60;
    return ticksPerSecond;

}



}
