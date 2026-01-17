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
public double distanceFromRed(double robotX, double robotY){
    double targetX = 144;
    double targetY = 144;
    double distance = Math.sqrt(Math.pow(targetX-robotX,2)+Math.pow(targetY-robotY,2));
    return distance;
}
    public double distanceFromBlue(double robotX, double robotY){
        double targetX = 0;
        double targetY = 144;
        double distance = Math.sqrt(Math.pow(targetX-robotX,2)+Math.pow(targetY-robotY,2));
        return distance;
    }
    public double getthetared(double robotX, double robotY) {
        return Math.atan2(144-robotY, 144-robotX);
    }
    public double getthetablue(double x1, double y1) {
        return Math.atan2(0-x1, 144-y1);
    }
    public double autoshoot(double x1, double y1, boolean isRed){
    double distance;
    if(isRed){
     distance = distanceFromRed(x1,y1);
    }else{
         distance = distanceFromBlue(x1,y1);
    }
        double RPM = -0.0000869661*Math.pow(distance,4)+0.0415709*Math.pow(distance,3)-7.21082*Math.pow(distance,2)+542.97966*distance-12464.0027;
    return RPM + 70;
    }
}


