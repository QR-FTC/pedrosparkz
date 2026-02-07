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
        return Math.toDegrees(Math.atan2(144-robotY, 144-robotX));
    }
    public double getthetablue(double x1, double y1) {
        return Math.toDegrees(Math.atan2(144-y1, 0-x1));
    }
    public double autoshoot(double x1, double y1, boolean isRed){
    double distance;
    if(isRed){
     distance = distanceFromRed(x1,y1);
    }else{
         distance = distanceFromBlue(x1,y1);
    }
        double RPM = -0.00025991*Math.pow(distance,4)+0.102756*Math.pow(distance,3)-14.88652*Math.pow(distance,2)+940.69624*distance-19241.3998;
    return RPM + 70;
    }
}


