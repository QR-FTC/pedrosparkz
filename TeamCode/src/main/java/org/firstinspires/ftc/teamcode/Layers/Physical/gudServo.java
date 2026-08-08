package org.firstinspires.ftc.teamcode.Layers.Physical;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class gudServo {
    Servo Servoervodervoservo;
    int xMin = 0;
    int xMax = 1;
    int yMin = 0;
    int yMax = 360;
    public gudServo(String name, HardwareMap hardwareMap){
        Servoervodervoservo = hardwareMap.get(Servo.class, name);
    }
    public int mapRange(int value) {
        return ((value - xMin) * (yMax - yMin)) / (xMax - xMin) + yMin;
    }
    public void ChangeX(int a, int b){
        xMin = a;
        xMax = b;
    }
    public void ChangeY(int c, int d){
        yMin = c;
        yMax = d;
    }
    public double position(){
        return Servoervodervoservo.getPosition();
    }
}
